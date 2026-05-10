#include "orca_decision/behavior_tree_nodes.hpp"

FinishMission::FinishMission(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList FinishMission::providedPorts() {
    return {};
}

BT::NodeStatus FinishMission::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    if (ctx_) {
        ctx_->current_action = name();
        ctx_->debug_msg = "Mission finished - surfacing";

        // Stop all motion
        ctx_->wrench_adapter->setCommand(MotionCommand());

        // Surface: set depth to 0
        if (ctx_->desired_depth_pub) {
            std_msgs::msg::Float64 msg;
            msg.data = 0.0;
            ctx_->desired_depth_pub->publish(msg);
        }

        RCLCPP_INFO(ctx_->node->get_logger(), "FinishMission: surfacing, mission complete.");
    }

    return BT::NodeStatus::SUCCESS;
}
