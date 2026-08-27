#include "orca_decision/behavior_tree_nodes.hpp"
#include <algorithm>

namespace {
constexpr float kBlindSurge = 8.0f;   // used while depth is unresolved
constexpr float kMaxSurge = 20.0f;
// Smallest surge that actually moves the vehicle. thruster_force_to_pwm sends
// PWM 1500 (no thrust) for any per-thruster force <= 0.392 N; pure surge splits
// evenly across the four +/-45 deg horizontals, so force.x = 1.11 N is the
// deadband edge, i.e. surge 1.39 at k_surge 0.8. 2.0 clears it by ~1.4x.
// Without this floor the command decays below the deadband while still outside
// the success threshold and the node returns RUNNING forever. Note this is not
// the old 5.0 floor: that one also applied when we had overshot, which is what
// turned "already too close" into a full-speed charge.
constexpr float kMinSurge = 2.0f;
// Ticks to keep closing in on a target whose depth never resolves before giving
// up. The BT ticks at 10 Hz (decision_node.cpp), so 300 is 30 s.
constexpr int kMaxBlindFrames = 300;
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

  // Single source of truth for the detection image centre — see decision_node's
  // declaration. Do not reintroduce a local constant here.
  const float centre_x =
      static_cast<float>(ctx_->node->get_parameter("image_center_x").as_double());
  const float centre_y =
      static_cast<float>(ctx_->node->get_parameter("image_center_y").as_double());

  auto obj = ctx_->world_model->getObjectNearestImageCenter(label, centre_x, centre_y);
  if (!obj.has_value()) {
    lost_frames_++;
    ctx_->debug_msg = "Target lost: " + label + " (" + std::to_string(lost_frames_) + "/8)";
    if (lost_frames_ > 8) {
      lost_frames_ = 0;
      blind_frames_ = 0;
      stable_frames_ = 0;
      ctx_->wrench_adapter->setCommand(MotionCommand());
      return BT::NodeStatus::FAILURE;
    }
    ctx_->wrench_adapter->setCommand(MotionCommand());
    return BT::NodeStatus::RUNNING;
  }
  
  lost_frames_ = 0;

  float error_x = centre_x - obj->cx;

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

  // Yaw correction with deadband and clamping to avoid aggressive oscillation
  if (std::abs(error_x) < 15.0f) {
    cmd.yaw = 0.0f; // within deadband, zero command
  } else {
    cmd.yaw = std::clamp(-0.0035f * error_x, -0.3f, 0.3f);
  }

  // Forward movement (decelerate as we get closer).
  if (obj->distance <= 0.0f) {
    blind_frames_++;
    ctx_->debug_msg = "Approaching (visual): " + label + " (h=" +
                      std::to_string(obj->height) + ", " +
                      std::to_string(blind_frames_) + "/" +
                      std::to_string(kMaxBlindFrames) + ")";
    if (blind_frames_ > kMaxBlindFrames) {
      blind_frames_ = 0;
      stable_frames_ = 0;
      ctx_->wrench_adapter->setCommand(MotionCommand());
      return BT::NodeStatus::FAILURE;
    }
    if (std::abs(error_x) > 60.0f) {
      cmd.surge = 0.0f; // Turn first if significantly off-center
    } else {
      cmd.surge = kBlindSurge;
    }
  } else {
    blind_frames_ = 0;
    const float dist_error = obj->distance - target_distance;
    if (dist_error <= 0.0f) {
      // Overshot: coast to a stop instead of pushing further in.
      cmd.surge = 0.0f;
    } else if (std::abs(error_x) > 60.0f) {
      // Too far off-center: turn first, don't drive in an arc
      cmd.surge = 0.0f;
    } else {
      cmd.surge = std::clamp(8.0f * dist_error, kMinSurge, kMaxSurge);
    }
  }

  ctx_->wrench_adapter->setCommand(cmd);

  return BT::NodeStatus::RUNNING;
}

void ApproachTarget::halt() {
  // Every counter, not just stable_frames_: halt() is what the enclosing
  // Timeout calls on expiry, and RetryUntilSuccessful re-enters the node right
  // afterwards. A carried-over blind_frames_ or lost_frames_ would make the
  // next attempt give up early on a count it did not earn.
  stable_frames_ = 0;
  lost_frames_ = 0;
  blind_frames_ = 0;
  if (ctx_) {
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}
