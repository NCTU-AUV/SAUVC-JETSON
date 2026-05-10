#include "orca_decision/behavior_tree_nodes.hpp"
#include <algorithm>
#include <cmath>

MoveAboveTarget::MoveAboveTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList MoveAboveTarget::providedPorts() {
    return { BT::InputPort<std::string>("label") };
}

BT::NodeStatus MoveAboveTarget::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    std::string label;
    if (!getInput<std::string>("label", label)) {
        throw BT::RuntimeError("missing required input [label]");
    }

    ctx_->current_action = name();
    ctx_->target_label = label;

    // Get parameters from ROS node
    double k_surge = ctx_->node->get_parameter("move_above_k_surge").as_double();
    double k_sway = ctx_->node->get_parameter("move_above_k_sway").as_double();
    double max_surge = ctx_->node->get_parameter("move_above_max_surge").as_double();
    double max_sway = ctx_->node->get_parameter("move_above_max_sway").as_double();
    double reacquire_surge =
        ctx_->node->get_parameter("move_above_reacquire_surge").as_double();
    double center_threshold = ctx_->node->get_parameter("move_above_center_threshold").as_double();
    int required_stable = ctx_->node->get_parameter("move_above_stable_frames").as_int();
    double timeout_sec = ctx_->node->get_parameter("move_above_timeout_sec").as_double();
    double lp_alpha = ctx_->node->get_parameter("move_above_lowpass_alpha").as_double();
    double center_x = ctx_->node->get_parameter("bottom_cam_center_x").as_double();
    double center_y = ctx_->node->get_parameter("bottom_cam_center_y").as_double();

    rclcpp::Time now = ctx_->node->now();

    if (!started_) {
        started_ = true;
        start_time_ = now;
        target_acquired_ = false;
        stable_frames_ = 0;
        filtered_error_x_ = 0.0f;
        filtered_error_y_ = 0.0f;
    }

    // Timeout check
    double elapsed = (now - start_time_).seconds();
    if (elapsed >= timeout_sec) {
        started_ = false;
        target_acquired_ = false;
        ctx_->debug_msg = "MoveAboveTarget timeout";
        ctx_->wrench_adapter->setCommand(MotionCommand());
        return BT::NodeStatus::FAILURE;
    }

    auto obj = ctx_->world_model->getBestObject(label);
    if (!target_acquired_) {
        if (!obj.has_value()) {
            stable_frames_ = 0;
            filtered_error_x_ = 0.0f;
            filtered_error_y_ = 0.0f;

            MotionCommand cmd;
            cmd.surge =
                std::clamp(static_cast<float>(reacquire_surge), 0.0f,
                           static_cast<float>(max_surge));
            ctx_->wrench_adapter->setCommand(cmd);
            ctx_->debug_msg =
                "MoveAboveTarget: standard reacquire, surging forward";
            return BT::NodeStatus::RUNNING;
        }

        target_acquired_ = true;
        stable_frames_ = 0;
        filtered_error_x_ = 0.0f;
        filtered_error_y_ = 0.0f;
    }

    if (!obj.has_value()) {
        target_acquired_ = false;
        stable_frames_ = 0;
        filtered_error_x_ = 0.0f;
        filtered_error_y_ = 0.0f;

        MotionCommand cmd;
        cmd.surge =
            std::clamp(static_cast<float>(reacquire_surge), 0.0f,
                       static_cast<float>(max_surge));
        ctx_->wrench_adapter->setCommand(cmd);
        ctx_->debug_msg =
            "MoveAboveTarget: surging forward to acquire target";
        return BT::NodeStatus::RUNNING;
    }

    // Compute raw errors from bottom camera bbox center
    // cx: left=small, right=large
    // cy: top=small, bottom=large
    float raw_error_x = obj->cx - static_cast<float>(center_x);
    float raw_error_y = obj->cy - static_cast<float>(center_y);

    // Low-pass filter to avoid bbox jitter → AUV oscillation
    filtered_error_x_ = static_cast<float>(lp_alpha) * raw_error_x
                       + (1.0f - static_cast<float>(lp_alpha)) * filtered_error_x_;
    filtered_error_y_ = static_cast<float>(lp_alpha) * raw_error_y
                       + (1.0f - static_cast<float>(lp_alpha)) * filtered_error_y_;

    ctx_->debug_msg = "MoveAbove err_x=" + std::to_string(filtered_error_x_)
                    + " err_y=" + std::to_string(filtered_error_y_);

    // Check if centered
    if (std::abs(filtered_error_x_) < center_threshold &&
        std::abs(filtered_error_y_) < center_threshold) {
        stable_frames_++;
        if (stable_frames_ >= required_stable) {
            started_ = false;
            target_acquired_ = false;
            ctx_->wrench_adapter->setCommand(MotionCommand());
            ctx_->debug_msg = "MoveAboveTarget: centered";
            return BT::NodeStatus::SUCCESS;
        }
    } else {
        stable_frames_ = 0;
    }

    // Match the mission spec's bottom-camera control mapping.
    MotionCommand cmd;
    cmd.sway = std::clamp(static_cast<float>(k_sway * filtered_error_x_),
                          static_cast<float>(-max_sway), static_cast<float>(max_sway));
    cmd.surge = std::clamp(static_cast<float>(-k_surge * filtered_error_y_),
                           static_cast<float>(-max_surge), static_cast<float>(max_surge));

    ctx_->wrench_adapter->setCommand(cmd);
    return BT::NodeStatus::RUNNING;
}

void MoveAboveTarget::halt() {
    started_ = false;
    target_acquired_ = false;
    stable_frames_ = 0;
    filtered_error_x_ = 0.0f;
    filtered_error_y_ = 0.0f;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
