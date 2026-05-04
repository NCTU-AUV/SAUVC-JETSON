#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

TurnToYaw::TurnToYaw(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList TurnToYaw::providedPorts() {
    return { BT::InputPort<double>("angle") };
}

BT::NodeStatus TurnToYaw::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    double target_angle;
    if (!getInput<double>("angle", target_angle)) {
        throw BT::RuntimeError("missing required input [angle]");
    }

    ctx_->current_action = name();
    ctx_->target_label = "";
    ctx_->debug_msg = "Turn to yaw: " + std::to_string(target_angle);

    double current_yaw = ctx_->world_model->getYaw();
    double error = normalizeAngle(target_angle - current_yaw);

    if (std::abs(error) < ctx_->align_yaw_threshold) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
        return BT::NodeStatus::SUCCESS;
    }

    MotionCommand cmd;
    cmd.yaw = 1.0f * error;
    ctx_->wrench_adapter->setCommand(cmd);

    return BT::NodeStatus::RUNNING;
}

void TurnToYaw::halt() {
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
