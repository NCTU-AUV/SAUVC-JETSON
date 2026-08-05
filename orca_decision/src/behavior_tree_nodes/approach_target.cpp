#include "orca_decision/behavior_tree_nodes.hpp"
#include <algorithm>

namespace {
// Detections live in the 640x640 YOLO tensor space, so the image centre is
// (320, 320) on both axes — 240 would be right only for a 640x480 image.
constexpr float kImageCentreX = 320.0f;
constexpr float kImageCentreY = 320.0f;
constexpr float kBlindSurge = 8.0f;   // used while depth is unresolved
constexpr float kMaxSurge = 20.0f;
}  // namespace

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

  auto obj = ctx_->world_model->getObjectNearestImageCenter(label, kImageCentreX, kImageCentreY);
  if (!obj.has_value()) {
    lost_frames_++;
    ctx_->debug_msg = "Target lost: " + label + " (" + std::to_string(lost_frames_) + "/8)";
    if (lost_frames_ > 8) {
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

  // Yaw correction from the box centre. cx is an absolute pixel column in the
  // 640-wide tensor. A target right of centre gives error_x < 0 and therefore
  // a positive yaw command, which is a starboard turn in this stack's
  // down-positive frame — towards the target. (Verified against the simulator:
  // positive torque.z yaws the vehicle to starboard.)
  float error_x = kImageCentreX - obj->cx;
  cmd.yaw = -0.005f * error_x; // simple P control

  // Forward movement (decelerate as we get closer).
  //
  // The three cases have to stay separate. Folding "no depth yet" and "already
  // too close" into a single `dist_error = 1000` mapped both onto *maximum*
  // surge — so being closer than requested commanded a full-speed charge into
  // the target, and in usb camera mode (distance is always -1.0 there) the
  // vehicle drove blind at full speed for the node's whole lifetime.
  if (obj->distance <= 0.0f) {
    // Depth not resolved yet — close in gently so the target grows in frame
    // until the depth estimate becomes valid, rather than charging at it.
    cmd.surge = kBlindSurge;
  } else {
    const float dist_error = obj->distance - target_distance;
    // Negative error means we overshot: clamping the low end at 0 lets the
    // vehicle coast to a stop instead of pushing further in.
    cmd.surge = std::clamp(8.0f * dist_error, 0.0f, kMaxSurge);
  }

  ctx_->wrench_adapter->setCommand(cmd);

  return BT::NodeStatus::RUNNING;
}

void ApproachTarget::halt() {
  stable_frames_ = 0;
  if (ctx_) {
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}
