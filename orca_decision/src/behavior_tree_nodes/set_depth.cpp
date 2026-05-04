#include "orca_decision/behavior_tree_nodes.hpp"

SetDepth::SetDepth(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList SetDepth::providedPorts() {
    return { BT::InputPort<float>("depth") };
}

BT::NodeStatus SetDepth::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    float depth;
    if (!getInput<float>("depth", depth)) {
        throw BT::RuntimeError("missing required input [depth]");
    }

    if (ctx_) {
        ctx_->current_action = name();
        ctx_->debug_msg = "Set depth: " + std::to_string(depth);
    }

    if (ctx_ && ctx_->desired_depth_pub) {
        std_msgs::msg::Float64 msg;
        msg.data = depth;
        ctx_->desired_depth_pub->publish(msg);
    }

    return BT::NodeStatus::SUCCESS;
}
