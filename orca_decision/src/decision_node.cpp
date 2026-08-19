#include "orca_decision/decision_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>

// Forward declarations of BT Node registration (to be implemented)
extern void RegisterBehaviorTreeNodes(BT::BehaviorTreeFactory &factory,
                                      std::shared_ptr<DecisionContext> ctx);

namespace {

std::string jsonEscape(const std::string &in) {
  std::string out;
  out.reserve(in.size());
  for (const char c : in) {
    switch (c) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      if (static_cast<unsigned char>(c) < 0x20) {
        char buf[7];
        std::snprintf(buf, sizeof(buf), "\\u%04x",
                      static_cast<unsigned int>(static_cast<unsigned char>(c)));
        out += buf;
      } else {
        out += c;
      }
    }
  }
  return out;
}

// JSON has no NaN literal and the browser's JSON.parse rejects one, so a
// non-finite value has to go out as null. target_position is NaN on every tick
// with no target lock — by far the common case — and emitting it verbatim would
// make the GUI drop the whole frame rather than one field.
std::string jsonNumber(double value, int places = 3) {
  if (!std::isfinite(value)) {
    return "null";
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", places, value);
  return buf;
}

std::string jsonBool(bool value) { return value ? "true" : "false"; }

}  // namespace

DecisionNode::DecisionNode() : Node("decision_node") {
  ctx_ = std::make_shared<DecisionContext>();
  ctx_->node = this;

  loadParameters();
  setupInterfaces();

  // We expect tree XML to be passed in from launch file or param
  registerBehaviorTree();

  // 10 Hz BT tick
  bt_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&DecisionNode::btTickLoop, this));

  // 50 Hz Wrench publish
  wrench_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&DecisionNode::publishWrenchLoop, this));

  RCLCPP_INFO(this->get_logger(), "DecisionNode initialized.");
}

void DecisionNode::loadParameters() {
  this->declare_parameter("k_surge", 1.0);
  this->declare_parameter("k_sway", 1.0);
  this->declare_parameter("k_yaw", 0.5);
  this->declare_parameter("motion_lowpass_alpha", 0.2);

  this->declare_parameter("max_velocity_clamp", 1.5);
  this->declare_parameter("perception_timeout_sec", 1.0);
  this->declare_parameter("velocity_decay", 0.95);

  this->declare_parameter("tree_xml_file", "config/trees.xml");
  this->declare_parameter("main_tree_id", "FinalMission");

  this->declare_parameter("camera_switch_hysteresis", 0.5);
  this->declare_parameter("align_yaw_threshold", 0.1);
  this->declare_parameter("align_distance_threshold", 0.5);

  // MoveAboveTarget parameters
  this->declare_parameter("move_above_k_surge", 0.003);
  this->declare_parameter("move_above_k_sway", 0.003);
  this->declare_parameter("move_above_max_surge", 0.15);
  this->declare_parameter("move_above_max_sway", 0.15);
  this->declare_parameter("move_above_reacquire_surge", 0.10);
  this->declare_parameter("move_above_center_threshold", 20.0);
  this->declare_parameter("move_above_stable_frames", 10);
  this->declare_parameter("move_above_timeout_sec", 30.0);
  this->declare_parameter("move_above_lowpass_alpha", 0.3);
  this->declare_parameter("check_bottom_clear_surge_speed", 0.2);
  this->declare_parameter("check_bottom_clear_stable_frames", 3);
  this->declare_parameter("check_bottom_clear_timeout_sec", 10.0);
  this->declare_parameter("check_bottom_clear_center_deadband", 20.0);

  // Detection image centre, in pixels.
  //
  // Detections arrive in the 640x640 YOLO tensor space (yolov8_decoder writes
  // raw tensor coordinates and depth_perception passes them through unscaled),
  // so the centre is (320, 320) on both axes and is the same for the front and
  // bottom cameras — there is only one centre in this stack. The old
  // bottom_cam_center_y of 240 was right only for a 640x480 image and put a
  // permanent 80 px offset on every MoveAboveTarget ball drop.
  //
  // One parameter pair, read by every node that needs it. It used to be four
  // separate spellings: this pair, two anonymous-namespace constants in
  // approach_target.cpp and final_align_target.cpp, and bare literals in
  // avoid_obstacle.cpp — so fixing 240 in one place fixed nothing elsewhere.
  this->declare_parameter("image_center_x", 320.0);
  this->declare_parameter("image_center_y", 320.0);

  // BumpFlare parameters
  this->declare_parameter("bump_flare_surge", 0.25);
  this->declare_parameter("bump_flare_timeout_sec", 8.0);

  // WaitForFlareOrder parameters
  this->declare_parameter("wait_for_flare_order_timeout_sec", 15.0);
  this->declare_parameter("wait_for_flare_order_default_order", "rby");

  // GoToPose parameters
  this->declare_parameter("go_to_pose_surge", 0.3);
  this->declare_parameter("go_to_pose_threshold", 1.0);
  this->declare_parameter("go_to_pose_timeout_sec", 30.0);

  // SearchBottomTarget parameters
  this->declare_parameter("search_bottom_yaw_speed", 0.2);
  this->declare_parameter("search_bottom_timeout_sec", 30.0);

  // SpiralSearch parameters
  this->declare_parameter("spiral_search_timeout_sec", 45.0);

  // Actuator timing
  this->declare_parameter("drop_ball_wait_sec", 0.5);
  this->declare_parameter("actuator_wait_sec", 1.0);

  // Rate of the JSON status mirror, in Hz. Only a human reads it, so it is
  // decimated off the 50 Hz wrench loop; 5 Hz is fast enough to watch the tree
  // move between nodes without pushing 50 websocket frames a second into a
  // browser that repaints a handful of text fields with them.
  this->declare_parameter("status_json_rate_hz", 5.0);
  const double status_json_rate_hz =
      this->get_parameter("status_json_rate_hz").as_double();
  constexpr double kWrenchLoopHz = 50.0;
  status_json_decimation_ =
      status_json_rate_hz > 0.0
          ? std::max(1, static_cast<int>(std::lround(kWrenchLoopHz /
                                                     status_json_rate_hz)))
          : 0;  // 0 disables the mirror entirely

  float k_surge = this->get_parameter("k_surge").as_double();
  float k_sway = this->get_parameter("k_sway").as_double();
  float k_yaw = this->get_parameter("k_yaw").as_double();
  float alpha = this->get_parameter("motion_lowpass_alpha").as_double();

  ctx_->wrench_adapter =
      std::make_shared<WrenchControllerAdapter>(k_surge, k_sway, k_yaw, alpha);

  double vel_decay = this->get_parameter("velocity_decay").as_double();
  double max_vel = this->get_parameter("max_velocity_clamp").as_double();
  double timeout = this->get_parameter("perception_timeout_sec").as_double();

  ctx_->world_model =
      std::make_shared<WorldModel>(this, vel_decay, max_vel, timeout);

  tree_xml_file_ = this->get_parameter("tree_xml_file").as_string();
  main_tree_id_ = this->get_parameter("main_tree_id").as_string();

  ctx_->align_yaw_threshold =
      this->get_parameter("align_yaw_threshold").as_double();
  ctx_->align_distance_threshold =
      this->get_parameter("align_distance_threshold").as_double();
}

void DecisionNode::setupInterfaces() {
  auto default_qos = rclcpp::QoS(10);
  auto command_qos = rclcpp::QoS(1).reliable();
  auto camera_mode_qos = rclcpp::QoS(1).reliable().transient_local();

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/orca/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&DecisionNode::imuCallback, this, std::placeholders::_1));

  perception_sub_ =
      this->create_subscription<orca_interface::msg::PerceptionArray>(
          "/orca/perception_array", default_qos,
          std::bind(&DecisionNode::perceptionCallback, this,
                    std::placeholders::_1));

  start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/orca/decision/start_mission", default_qos,
      std::bind(&DecisionNode::startMissionCallback, this,
                std::placeholders::_1));

  flare_order_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/orca/decision/flare_order", default_qos,
      std::bind(&DecisionNode::flareOrderCallback, this,
                std::placeholders::_1));

  wrench_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(
      "/orca/decision/wrench", default_qos);
  desired_depth_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/orca/decision/desired_depth", default_qos);
  camera_mode_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/orca/decision/camera_mode", camera_mode_qos);
  arm_pub_ =
      this->create_publisher<std_msgs::msg::Int32>("/orca/decision/arm",
                                                   command_qos);
  hand_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/orca/decision/hand",
                                                   command_qos);
  status_pub_ = this->create_publisher<orca_interface::msg::DecisionStatus>(
      "/orca/decision/status", default_qos);
  // Deliberately not remapped in decision.launch.py: the Web GUI reaches this
  // topic the same way it reaches /orca/decision/start_mission — absolutely,
  // across the container boundary. The two stacks share one ROS graph but not
  // one namespace, and the GUI is the only consumer.
  status_json_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/orca/decision/status_json", default_qos);

  ctx_->camera_mode_pub = camera_mode_pub_;
  ctx_->desired_depth_pub = desired_depth_pub_;
  ctx_->arm_pub = arm_pub_;
  ctx_->hand_pub = hand_pub_;
}

void DecisionNode::registerBehaviorTree() {
  RegisterBehaviorTreeNodes(bt_factory_, ctx_);

  // Attempt to load absolute or relative
  std::string path = tree_xml_file_;
  if (path.front() != '/') {
    std::string pkg_share =
        ament_index_cpp::get_package_share_directory("orca_decision");
    path = pkg_share + "/" + path;
  }

  try {
    bt_factory_.registerBehaviorTreeFromFile(path);

    auto blackboard = BT::Blackboard::create();
    blackboard->set("ctx", ctx_);

    tree_ = std::make_unique<BT::Tree>(
        bt_factory_.createTree(main_tree_id_, blackboard));
    RCLCPP_INFO(this->get_logger(), "BehaviorTree %s loaded successfully.",
                main_tree_id_.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load BehaviorTree: %s",
                 e.what());
  }
}

void DecisionNode::btTickLoop() {
  if (!ctx_->mission_started || mission_complete_) {
    return;
  }
  if (tree_) {
    auto status = tree_->tickRoot();
    if (status == BT::NodeStatus::SUCCESS) {
      mission_complete_ = true;
      ctx_->mission_started = false;
      ctx_->current_action = "MissionComplete";
      ctx_->debug_msg = "Mission finished successfully";
      ctx_->wrench_adapter->setCommand(MotionCommand());
      RCLCPP_INFO(this->get_logger(), "Mission completed successfully.");
    } else if (status == BT::NodeStatus::FAILURE) {
      mission_complete_ = true;
      ctx_->mission_started = false;
      ctx_->current_action = "MissionFailed";
      ctx_->debug_msg = "Mission ended with FAILURE";
      ctx_->wrench_adapter->setCommand(MotionCommand());
      RCLCPP_WARN(this->get_logger(), "Mission ended with FAILURE.");
    }
  }
}

void DecisionNode::publishWrenchLoop() {
  // Runs whether or not the mission is up, unlike everything below it. The
  // DecisionStatus topic goes silent the instant the tree stops, which leaves
  // "not started yet", "finished a minute ago" and "the node died" looking
  // identical from the GUI. The mirror distinguishes all three.
  publishStatusJson();

  if (!ctx_->mission_started) {
    // Publish zero wrench
    geometry_msgs::msg::Wrench msg;
    wrench_pub_->publish(msg);
    return;
  }

  auto wrench_msg = ctx_->wrench_adapter->getWrench();
  wrench_pub_->publish(wrench_msg);

  status_pub_->publish(buildStatus());
}

orca_interface::msg::DecisionStatus DecisionNode::buildStatus() {
  orca_interface::msg::DecisionStatus status;
  status.header.stamp = this->now();
  status.header.frame_id = "auv";
  status.mission_phase = main_tree_id_;
  status.current_action = ctx_->current_action;
  status.target_label = ctx_->target_label;

  if (!ctx_->target_label.empty()) {
    auto obj = ctx_->world_model->getBestObject(ctx_->target_label);
    if (obj) {
      status.target_position.x = obj->position_world.x();
      status.target_position.y = obj->position_world.y();
      status.target_position.z = obj->position_world.z();
      status.target_locked = true;
    } else {
      status.target_position.x = std::numeric_limits<double>::quiet_NaN();
      status.target_position.y = std::numeric_limits<double>::quiet_NaN();
      status.target_position.z = std::numeric_limits<double>::quiet_NaN();
      status.target_locked = false;
    }
  } else {
    status.target_position.x = std::numeric_limits<double>::quiet_NaN();
    status.target_position.y = std::numeric_limits<double>::quiet_NaN();
    status.target_position.z = std::numeric_limits<double>::quiet_NaN();
    status.target_locked = false;
  }

  status.is_recovering = ctx_->is_recovering;
  if (ctx_->mission_started) {
    status.mission_time = (this->now() - ctx_->mission_start_time).seconds();
  } else {
    status.mission_time = 0.0;
  }
  status.camera_mode = ctx_->camera_mode;
  status.debug = ctx_->debug_msg;

  return status;
}

void DecisionNode::publishStatusJson() {
  if (status_json_decimation_ <= 0) {
    return;
  }
  if (++status_json_counter_ < status_json_decimation_) {
    return;
  }
  status_json_counter_ = 0;

  const auto status = buildStatus();

  // Hand-rolled rather than pulled in as a dependency: this is one flat object
  // of known fields, and orca_decision has no JSON library in its build already.
  std::ostringstream json;
  json << "{"
       << "\"mission_started\":" << jsonBool(ctx_->mission_started)
       // Distinguishes a tree that ran to its end from one that never started;
       // current_action carries which of SUCCESS/FAILURE it was.
       << ",\"mission_complete\":" << jsonBool(mission_complete_)
       << ",\"mission_phase\":\"" << jsonEscape(status.mission_phase) << "\""
       << ",\"current_action\":\"" << jsonEscape(status.current_action) << "\""
       << ",\"target_label\":\"" << jsonEscape(status.target_label) << "\""
       << ",\"target_locked\":" << jsonBool(status.target_locked)
       << ",\"target_position\":{"
       << "\"x\":" << jsonNumber(status.target_position.x)
       << ",\"y\":" << jsonNumber(status.target_position.y)
       << ",\"z\":" << jsonNumber(status.target_position.z) << "}"
       << ",\"is_recovering\":" << jsonBool(status.is_recovering)
       << ",\"mission_time\":" << jsonNumber(status.mission_time, 1)
       << ",\"camera_mode\":\"" << jsonEscape(status.camera_mode) << "\""
       << ",\"debug\":\"" << jsonEscape(status.debug) << "\""
       << "}";

  std_msgs::msg::String msg;
  msg.data = json.str();
  status_json_pub_->publish(msg);
}

void DecisionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  ctx_->world_model->updateFromIMU(msg);
}

void DecisionNode::perceptionCallback(
    const orca_interface::msg::PerceptionArray::SharedPtr msg) {
  ctx_->world_model->updateFromPerception(msg);
}

void DecisionNode::startMissionCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  ctx_->mission_started = msg->data;
  if (ctx_->mission_started) {
    mission_complete_ = false;  // allow BT to run again
    ctx_->mission_start_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Mission Started!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Mission Stopped!");
    // Reset command
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}

void DecisionNode::flareOrderCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  ctx_->current_flare_order = msg->data;
}
