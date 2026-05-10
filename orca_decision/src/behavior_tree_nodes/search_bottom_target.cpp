#include "orca_decision/behavior_tree_nodes.hpp"

SearchBottomTarget::SearchBottomTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList SearchBottomTarget::providedPorts() {
    return { BT::InputPort<std::string>("label") };
}

BT::NodeStatus SearchBottomTarget::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    std::string label;
    if (!getInput<std::string>("label", label)) {
        throw BT::RuntimeError("missing required input [label]");
    }

    ctx_->current_action = name();
    ctx_->target_label = label;

    double yaw_speed = ctx_->node->get_parameter("search_bottom_yaw_speed").as_double();
    double timeout_sec = ctx_->node->get_parameter("search_bottom_timeout_sec").as_double();

    rclcpp::Time now = ctx_->node->now();

    if (!started_) {
        started_ = true;
        start_time_ = now;
    }

    // Timeout check
    double elapsed = (now - start_time_).seconds();
    if (elapsed >= timeout_sec) {
        started_ = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
        ctx_->debug_msg = "SearchBottomTarget: timeout";
        return BT::NodeStatus::FAILURE;
    }

    auto obj = ctx_->world_model->getBestObject(label);
    if (obj.has_value()) {
        started_ = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
        ctx_->debug_msg = "SearchBottomTarget: found " + label;
        return BT::NodeStatus::SUCCESS;
    }

    // Not found — slow yaw sweep to scan bottom camera FOV
    MotionCommand cmd;
    cmd.yaw = static_cast<float>(yaw_speed);
    ctx_->wrench_adapter->setCommand(cmd);
    ctx_->debug_msg = "SearchBottomTarget: sweeping for " + label;

    return BT::NodeStatus::RUNNING;
}

void SearchBottomTarget::halt() {
    started_ = false;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
