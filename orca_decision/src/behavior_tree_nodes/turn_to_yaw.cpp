#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

namespace {
constexpr double kPi = 3.14159265358979323846;
}

TurnToYaw::TurnToYaw(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList TurnToYaw::providedPorts() {
    return { BT::InputPort<double>("degrees") };
}

BT::NodeStatus TurnToYaw::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    double degrees;
    if (!getInput<double>("degrees", degrees)) {
        throw BT::RuntimeError("missing required input [degrees]");
    }

    if (!started_) {
        const double current_yaw = ctx_->world_model->getYaw();
        const double delta_yaw = degrees * kPi / 180.0;
        target_yaw_ = normalizeAngle(current_yaw + delta_yaw);
        started_ = true;
    }

    ctx_->current_action = name();
    ctx_->target_label = "";
    ctx_->debug_msg = "Turn yaw by " + std::to_string(degrees) + " deg";

    double current_yaw = ctx_->world_model->getYaw();
    double error = normalizeAngle(target_yaw_ - current_yaw);

    if (std::abs(error) < ctx_->align_yaw_threshold) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
        started_ = false;
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
    started_ = false;
}
