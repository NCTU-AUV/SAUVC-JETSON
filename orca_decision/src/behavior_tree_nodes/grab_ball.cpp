#include "orca_decision/behavior_tree_nodes.hpp"

GrabBall::GrabBall(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList GrabBall::providedPorts() {
    return {};
}

BT::NodeStatus GrabBall::tick() {
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
        // Publish hand = false (off / close / grab)
        if (ctx_->hand_pub) {
            std_msgs::msg::Bool msg;
            msg.data = false;
            ctx_->hand_pub->publish(msg);
        }
        ctx_->debug_msg = "GrabBall: hand off";
        RCLCPP_INFO(ctx_->node->get_logger(), "GrabBall: hand=false published");
        return BT::NodeStatus::RUNNING;
    }

    double elapsed = (now - start_time_).seconds();
    if (elapsed >= wait_sec) {
        started_ = false;
        ctx_->debug_msg = "GrabBall: done";
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void GrabBall::halt() {
    started_ = false;
}
