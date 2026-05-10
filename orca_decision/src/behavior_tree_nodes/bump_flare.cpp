#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>
#include <algorithm>

static inline double normalizeAngleBump(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

BumpFlare::BumpFlare(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList BumpFlare::providedPorts() {
    return { BT::InputPort<double>("timeout") };
}

BT::NodeStatus BumpFlare::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    double timeout = 8.0;
    getInput<double>("timeout", timeout);

    // Also read from param if available
    double param_timeout = ctx_->node->get_parameter("bump_flare_timeout_sec").as_double();
    if (param_timeout > 0.0) timeout = param_timeout;

    double bump_surge = ctx_->node->get_parameter("bump_flare_surge").as_double();

    ctx_->current_action = name();
    ctx_->debug_msg = "Bumping flare";

    rclcpp::Time now = ctx_->node->now();

    if (!started_) {
        started_ = true;
        start_time_ = now;
        target_yaw_ = ctx_->world_model->getYaw();
    }

    double elapsed = (now - start_time_).seconds();
    if (elapsed >= timeout) {
        started_ = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
        ctx_->debug_msg = "BumpFlare: completed";
        return BT::NodeStatus::SUCCESS;
    }

    // Surge only with heading lock
    MotionCommand cmd;
    cmd.surge = static_cast<float>(bump_surge);

    // Heading lock
    double current_yaw = ctx_->world_model->getYaw();
    double yaw_error = normalizeAngleBump(target_yaw_ - current_yaw);
    cmd.yaw = std::clamp(static_cast<float>(1.0 * yaw_error), -0.5f, 0.5f);

    ctx_->wrench_adapter->setCommand(cmd);
    ctx_->debug_msg = "BumpFlare: t=" + std::to_string(elapsed) + "/" + std::to_string(timeout);

    return BT::NodeStatus::RUNNING;
}

void BumpFlare::halt() {
    started_ = false;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
