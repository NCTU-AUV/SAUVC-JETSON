#include "orca_decision/behavior_tree_nodes.hpp"
#include <algorithm>

ApproachTarget::ApproachTarget(const std::string &name,
                               const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config) {}

BT::PortsList ApproachTarget::providedPorts() {
  return {BT::InputPort<std::string>("label"),
          BT::InputPort<double>("distance")};
}

BT::NodeStatus ApproachTarget::tick() {
  if (!ctx_) {
    config().blackboard->get("ctx", ctx_);
  }

  std::string label;
  double target_distance;
  if (!getInput<std::string>("label", label) ||
      !getInput<double>("distance", target_distance)) {
    throw BT::RuntimeError("missing required inputs");
  }

  ctx_->current_action = name();
  ctx_->target_label = label;

  auto obj = ctx_->world_model->getBestObject(label);
  if (!obj.has_value()) {
    lost_frames_++;
    ctx_->debug_msg = "Target lost: " + label + " (" + std::to_string(lost_frames_) + "/5)";
    if (lost_frames_ > 5) {
      lost_frames_ = 0;
      ctx_->wrench_adapter->setCommand(MotionCommand());
      return BT::NodeStatus::FAILURE;
    }
    // Maintain current command or stop? If we just return RUNNING without setting command, it keeps the last command or we can just send zero wrench.
    // The prompt just says return failure only if for more than 5 frames in sequence. We should return RUNNING in the meantime.
    ctx_->wrench_adapter->setCommand(MotionCommand()); // stop while lost but waiting? or don't set command? Setting to zero is safer.
    return BT::NodeStatus::RUNNING;
  }
  
  lost_frames_ = 0;

  ctx_->debug_msg = "Approaching: dist=" + std::to_string(obj->distance);

  // Check SUCCESS condition: distance < target_distance
  // (Assuming distance is valid, i.e., > 0)
  if (obj->distance > 0.0 && obj->distance < target_distance) {
    stable_frames_++;
    if (stable_frames_ >= 5) {
      ctx_->wrench_adapter->setCommand(MotionCommand());
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    stable_frames_ = 0;
  }

  // Approach logic
  MotionCommand cmd;

  // Yaw correction based on cx (assuming cx is normalized -1 to 1 or pixel
  // coords?) Prompt says "using bbox cx". If cx is in pixels, we need camera
  // center. Let's assume cx is normalized [-1, 1] or we just use a generic P
  // controller. If cx is pixel coordinate (e.g., 0 to 640), then center is 320.
  // In Isaac ROS YOLO output, cx is usually absolute pixels. Let's assume
  // 640x480. A safer assumption is cx is normalized or we use a basic P-control
  // if it's pixels. Let's assume cx is in pixels (640 width), error = 320 - cx.
  float error_x = 320.0f - obj->cx;
  cmd.yaw = -0.005f * error_x; // simple P control

  // Forward movement (decelerate as we get closer)
  float dist_error = obj->distance - target_distance;
  if (dist_error < 0 || dist_error > 2.5f)
    dist_error = 1000;
  cmd.surge = std::clamp(8.0f * dist_error, 5.0f,
                         20.0f); // min 0.2 surge to keep moving

  ctx_->wrench_adapter->setCommand(cmd);

  return BT::NodeStatus::RUNNING;
}

void ApproachTarget::halt() {
  stable_frames_ = 0;
  if (ctx_) {
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}
