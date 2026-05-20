#include "orca_decision/decision_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Forward declarations of BT Node registration (to be implemented)
extern void RegisterBehaviorTreeNodes(BT::BehaviorTreeFactory &factory,
                                      std::shared_ptr<DecisionContext> ctx);

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
  this->declare_parameter("check_bottom_clear_sway_speed", 0.2);
  this->declare_parameter("check_bottom_clear_stable_frames", 3);
  this->declare_parameter("check_bottom_clear_timeout_sec", 10.0);
  this->declare_parameter("check_bottom_clear_center_deadband", 20.0);

  // Bottom camera center (pixels)
  this->declare_parameter("bottom_cam_center_x", 320.0);
  this->declare_parameter("bottom_cam_center_y", 240.0);

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
      this->create_publisher<std_msgs::msg::Int32>("/orca/decision/hand",
                                                   command_qos);
  status_pub_ = this->create_publisher<orca_interface::msg::DecisionStatus>(
      "/orca/decision/status", default_qos);

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
  if (!ctx_->mission_started) {
    // Publish zero wrench
    geometry_msgs::msg::Wrench msg;
    wrench_pub_->publish(msg);
    return;
  }

  auto wrench_msg = ctx_->wrench_adapter->getWrench();
  wrench_pub_->publish(wrench_msg);

  // Also publish status
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

  status_pub_->publish(status);
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
