#include "orca_decision/behavior_tree_nodes.hpp"

WaitForStart::WaitForStart(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList WaitForStart::providedPorts() {
    return {};
}

BT::NodeStatus WaitForStart::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    ctx_->current_action = name();
    ctx_->target_label = "";

    // The DecisionNode already gates BT ticking on mission_started.
    // When this node is ticked, the mission IS started.
    // This node exists to make the BT XML self-documenting.
    if (ctx_->mission_started) {
        ctx_->debug_msg = "Mission started!";
        return BT::NodeStatus::SUCCESS;
    }

    ctx_->debug_msg = "Waiting for start signal...";
    return BT::NodeStatus::RUNNING;
}

void WaitForStart::halt() {
    // Nothing to clean up
}
