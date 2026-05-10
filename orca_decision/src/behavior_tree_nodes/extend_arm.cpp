#include "orca_decision/behavior_tree_nodes.hpp"

ExtendArm::ExtendArm(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList ExtendArm::providedPorts() {
    return {};
}

BT::NodeStatus ExtendArm::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    ctx_->current_action = name();
    ctx_->target_label = "";

    double wait_sec = ctx_->node->get_parameter("actuator_wait_sec").as_double();
    rclcpp::Time now = ctx_->node->now();

    if (!started_) {
        started_ = true;
        start_time_ = now;
        // Publish arm = 1 (extend)
        if (ctx_->arm_pub) {
            std_msgs::msg::Int32 msg;
            msg.data = 1;
            ctx_->arm_pub->publish(msg);
        }
        ctx_->debug_msg = "ExtendArm: extending";
        RCLCPP_INFO(ctx_->node->get_logger(), "ExtendArm: arm=1 published");
        return BT::NodeStatus::RUNNING;
    }

    double elapsed = (now - start_time_).seconds();
    if (elapsed >= wait_sec) {
        started_ = false;
        ctx_->debug_msg = "ExtendArm: done";
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void ExtendArm::halt() {
    started_ = false;
}
