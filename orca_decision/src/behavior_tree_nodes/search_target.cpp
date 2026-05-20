#include "orca_decision/behavior_tree_nodes.hpp"

SearchTarget::SearchTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList SearchTarget::providedPorts() {
    return { BT::InputPort<std::string>("label") };
}

BT::NodeStatus SearchTarget::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    std::string label;
    if (!getInput<std::string>("label", label)) {
        throw BT::RuntimeError("missing required input [label]");
    }

    ctx_->current_action = name();
    ctx_->target_label = label;
    ctx_->debug_msg = "Searching target: " + label;

    auto obj = ctx_->world_model->getBestObject(label);
    if (obj.has_value()) {
        ctx_->wrench_adapter->setCommand(MotionCommand()); // stop sweeping
        return BT::NodeStatus::SUCCESS;
    }

    // Not found, perform yaw sweep
    MotionCommand cmd;
    cmd.yaw = 2.0f; // Constant yaw sweep
    ctx_->wrench_adapter->setCommand(cmd);

    return BT::NodeStatus::RUNNING;
}

void SearchTarget::halt() {
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
