#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>

static double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

BlindForward::BlindForward(const std::string &name,
                           const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config) {}

BT::PortsList BlindForward::providedPorts() {
  return {BT::InputPort<double>("duration"),
          BT::InputPort<bool>("heading_lock")};
}

BT::NodeStatus BlindForward::tick() {
  if (!ctx_) {
    config().blackboard->get("ctx", ctx_);
  }

  double duration;
  bool heading_lock;
  if (!getInput<double>("duration", duration) ||
      !getInput<bool>("heading_lock", heading_lock)) {
    throw BT::RuntimeError("missing required inputs");
  }

  ctx_->current_action = name();
  ctx_->target_label = "";
  ctx_->debug_msg = "Blind forward";

  if (!started_) {
    started_ = true;
    start_time_ = ctx_->node->now();
    if (heading_lock) {
      target_yaw_ = ctx_->world_model->getYaw();
    }
  }

  auto elapsed = (ctx_->node->now() - start_time_).seconds();
  if (elapsed >= duration) {
    started_ = false;
    ctx_->wrench_adapter->setCommand(MotionCommand());
    return BT::NodeStatus::SUCCESS;
  }

  MotionCommand cmd;
  cmd.surge = 25.0f; // Constant surge

  if (heading_lock) {
    double current_yaw = ctx_->world_model->getYaw();
    double error = normalizeAngle(target_yaw_ - current_yaw);
    cmd.yaw = 1.0f * error; // P control for heading
  }

  ctx_->wrench_adapter->setCommand(cmd);

  return BT::NodeStatus::RUNNING;
}

void BlindForward::halt() {
  started_ = false;
  if (ctx_) {
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}
