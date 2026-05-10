#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>
#include <algorithm>

static inline double normalizeAngleGTP(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

GoToPose::GoToPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList GoToPose::providedPorts() {
    return { BT::InputPort<std::string>("pose_key", "drop_pose", "Blackboard key for target pose") };
}

BT::NodeStatus GoToPose::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    std::string pose_key = "drop_pose";
    getInput<std::string>("pose_key", pose_key);

    ctx_->current_action = name();
    ctx_->target_label = "";

    double surge_speed = ctx_->node->get_parameter("go_to_pose_surge").as_double();
    double threshold = ctx_->node->get_parameter("go_to_pose_threshold").as_double();
    double timeout_sec = ctx_->node->get_parameter("go_to_pose_timeout_sec").as_double();

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
        ctx_->debug_msg = "GoToPose: timeout";
        return BT::NodeStatus::FAILURE;
    }

    // Get target pose from blackboard
    Eigen::Vector3d target_pose;
    if (!config().blackboard->get(pose_key, target_pose)) {
        ctx_->debug_msg = "GoToPose: no pose in blackboard";
        ctx_->wrench_adapter->setCommand(MotionCommand());
        return BT::NodeStatus::FAILURE;
    }

    // Get current position via dead reckoning
    Eigen::Vector3d current_pos = ctx_->world_model->getAUVPosition();
    Eigen::Vector3d diff = target_pose - current_pos;
    double distance = diff.head<2>().norm();  // 2D distance (x, y)

    ctx_->debug_msg = "GoToPose: dist=" + std::to_string(distance);

    // SUCCESS: close enough (approximate, DR will drift)
    if (distance < threshold) {
        started_ = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
        ctx_->debug_msg = "GoToPose: arrived";
        return BT::NodeStatus::SUCCESS;
    }

    // Compute desired yaw toward target
    double desired_yaw = std::atan2(diff.y(), diff.x());
    double current_yaw = ctx_->world_model->getYaw();
    double yaw_error = normalizeAngleGTP(desired_yaw - current_yaw);

    MotionCommand cmd;
    cmd.yaw = std::clamp(static_cast<float>(1.0 * yaw_error), -0.5f, 0.5f);

    // Only surge forward when roughly pointing at target
    if (std::abs(yaw_error) < 0.5) {
        cmd.surge = static_cast<float>(surge_speed);
    } else {
        cmd.surge = 0.0f;  // Turn first, then move
    }

    ctx_->wrench_adapter->setCommand(cmd);
    return BT::NodeStatus::RUNNING;
}

void GoToPose::halt() {
    started_ = false;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
