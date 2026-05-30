#include "orca_decision/behavior_tree_nodes.hpp"

DropBall::DropBall(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList DropBall::providedPorts() {
    return {};
}

BT::NodeStatus DropBall::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    ctx_->current_action = name();
    ctx_->target_label = "";

    double wait_sec = ctx_->node->get_parameter("drop_ball_wait_sec").as_double();
    rclcpp::Time now = ctx_->node->now();

    if (!started_) {
        started_ = true;
        phase_ = 0;
        phase_start_ = now;
        // Phase 0: Stop AUV
        ctx_->wrench_adapter->setCommand(MotionCommand());
        ctx_->debug_msg = "DropBall: stopping AUV";
        return BT::NodeStatus::RUNNING;
    }

    double elapsed = (now - phase_start_).seconds();

    switch (phase_) {
        case 0: {
            // Wait 0.3s for AUV to settle
            if (elapsed >= 0.3) {
                phase_ = 1;
                phase_start_ = now;
                // Publish hand = true (on / open / release)
                if (ctx_->hand_pub) {
                    std_msgs::msg::Bool msg;
                    msg.data = false;
                    ctx_->hand_pub->publish(msg);
                }
                ctx_->debug_msg = "DropBall: hand on";
            }
            return BT::NodeStatus::RUNNING;
        }
        case 1: {
            // Wait for ball to drop
            if (elapsed >= wait_sec) {
                phase_ = 2;
                // Record current AUV pose to blackboard for TargetReacquisition
                Eigen::Vector3d pose = ctx_->world_model->getAUVPosition();
                config().blackboard->set("drop_pose", pose);
                ctx_->debug_msg = "DropBall: pose recorded ["
                    + std::to_string(pose.x()) + ", "
                    + std::to_string(pose.y()) + ", "
                    + std::to_string(pose.z()) + "]";

                RCLCPP_INFO(ctx_->node->get_logger(),
                    "DropBall: saved drop_pose (%.2f, %.2f, %.2f)",
                    pose.x(), pose.y(), pose.z());

                started_ = false;
                phase_ = 0;
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }
        default: {
            started_ = false;
            phase_ = 0;
            return BT::NodeStatus::FAILURE;
        }
    }
}

void DropBall::halt() {
    started_ = false;
    phase_ = 0;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
