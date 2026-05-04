#include "orca_decision/behavior_tree_nodes.hpp"

SetCamera::SetCamera(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList SetCamera::providedPorts() {
    return { BT::InputPort<std::string>("mode") };
}

BT::NodeStatus SetCamera::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    std::string mode;
    if (!getInput<std::string>("mode", mode)) {
        throw BT::RuntimeError("missing required input [mode]");
    }

    if (ctx_) {
        ctx_->current_action = name();
        ctx_->camera_mode = mode;
        ctx_->debug_msg = "Set camera: " + mode;
    }

    if (ctx_ && ctx_->camera_mode_pub) {
        std_msgs::msg::String msg;
        msg.data = mode;
        ctx_->camera_mode_pub->publish(msg);
    }

    return BT::NodeStatus::SUCCESS;
}
