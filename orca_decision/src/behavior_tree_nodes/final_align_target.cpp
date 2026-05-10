#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>

FinalAlignTarget::FinalAlignTarget(const std::string &name,
                                   const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config) {}

BT::PortsList FinalAlignTarget::providedPorts() {
  return {BT::InputPort<std::string>("label")};
}

BT::NodeStatus FinalAlignTarget::tick() {
  if (!ctx_) {
    config().blackboard->get("ctx", ctx_);
  }

  std::string label;
  if (!getInput<std::string>("label", label)) {
    throw BT::RuntimeError("missing required input [label]");
  }

  ctx_->current_action = name();
  ctx_->target_label = label;

  auto obj = ctx_->world_model->getBestObject(label);
  if (!obj.has_value()) {
    aligning_ = false;
    ctx_->debug_msg = "Target lost: " + label;
    ctx_->wrench_adapter->setCommand(MotionCommand());
    return BT::NodeStatus::FAILURE;
  }

  ctx_->debug_msg = "Aligning to: " + label;

  // Yaw error based on cx (assuming 640 width center is 320)
  float error_x = 320.0f - obj->cx;

  // Check if within threshold
  if (std::abs(error_x) < 10.0f) { // pixel threshold
    if (!aligning_) {
      aligning_ = true;
      align_start_time_ = ctx_->node->now();
    } else {
      auto elapsed = (ctx_->node->now() - align_start_time_).seconds();
      if (elapsed >= 0.1) {
        aligning_ = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
        return BT::NodeStatus::SUCCESS;
      }
    }
  } else {
    aligning_ = false;
  }

  // Align logic
  MotionCommand cmd;
  cmd.surge = 0.0f;
  cmd.yaw = -0.005f * error_x;

  ctx_->wrench_adapter->setCommand(cmd);

  return BT::NodeStatus::RUNNING;
}

void FinalAlignTarget::halt() {
  aligning_ = false;
  if (ctx_) {
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}
