#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>
#include <algorithm>

static inline double normalizeAngleSpiral(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

SpiralSearchBottom::SpiralSearchBottom(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList SpiralSearchBottom::providedPorts() {
    return {};
}

BT::NodeStatus SpiralSearchBottom::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    ctx_->current_action = name();
    ctx_->target_label = "";

    double timeout_sec = ctx_->node->get_parameter("spiral_search_timeout_sec").as_double();

    rclcpp::Time now = ctx_->node->now();

    if (!started_) {
        started_ = true;
        start_time_ = now;
        target_yaw_ = ctx_->world_model->getYaw();
    }

    double elapsed = (now - start_time_).seconds();

    // Timeout — return FAILURE so parent can handle
    if (elapsed >= timeout_sec) {
        started_ = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
        ctx_->debug_msg = "SpiralSearchBottom: timeout";
        return BT::NodeStatus::FAILURE;
    }

    // Check if ball appeared during spiral
    auto obj = ctx_->world_model->getBestObject("ball");
    if (obj.has_value()) {
        started_ = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
        ctx_->debug_msg = "SpiralSearchBottom: found ball!";
        return BT::NodeStatus::SUCCESS;
    }

    // Spiral motion: ultra-slow surge + slow oscillating sway + controlled heading drift
    // The heading slowly rotates to cover area, creating a spiral pattern
    double phase = elapsed * 0.3;  // slow phase progression

    MotionCommand cmd;
    cmd.surge = 0.08f;  // ultra-slow forward
    cmd.sway = static_cast<float>(0.06 * std::sin(phase));  // gentle sway oscillation

    // Heading lock with slow drift: heading target rotates at ~0.1 rad/s
    double drifting_target = target_yaw_ + elapsed * 0.1;
    double current_yaw = ctx_->world_model->getYaw();
    double yaw_error = normalizeAngleSpiral(drifting_target - current_yaw);
    cmd.yaw = std::clamp(static_cast<float>(0.5 * yaw_error), -0.3f, 0.3f);

    ctx_->wrench_adapter->setCommand(cmd);
    ctx_->debug_msg = "SpiralSearchBottom: t=" + std::to_string(elapsed);

    return BT::NodeStatus::RUNNING;
}

void SpiralSearchBottom::halt() {
    started_ = false;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
