#include "orca_decision/behavior_tree_nodes.hpp"
#include <algorithm>
#include <cmath>

CheckBottomClear::CheckBottomClear(const std::string& name,
                                   const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList CheckBottomClear::providedPorts() {
    return {};
}

BT::NodeStatus CheckBottomClear::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    ctx_->current_action = name();
    ctx_->target_label.clear();

    if (ctx_->camera_mode != "usb") {
        ctx_->debug_msg = "CheckBottomClear requires usb camera mode";
        ctx_->wrench_adapter->setCommand(MotionCommand());
        started_ = false;
        clear_frames_ = 0;
        return BT::NodeStatus::FAILURE;
    }

    const double timeout_sec =
        ctx_->node->get_parameter("check_bottom_clear_timeout_sec").as_double();
    const double sway_speed =
        ctx_->node->get_parameter("check_bottom_clear_sway_speed").as_double();
    const int required_clear_frames =
        ctx_->node->get_parameter("check_bottom_clear_stable_frames").as_int();
    const double center_x =
        ctx_->node->get_parameter("bottom_cam_center_x").as_double();
    const double center_deadband =
        ctx_->node->get_parameter("check_bottom_clear_center_deadband").as_double();

    const rclcpp::Time now = ctx_->node->now();
    if (!started_) {
        started_ = true;
        start_time_ = now;
        clear_frames_ = 0;
    }

    if ((now - start_time_).seconds() >= timeout_sec) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
        ctx_->debug_msg = "CheckBottomClear: timeout";
        started_ = false;
        clear_frames_ = 0;
        return BT::NodeStatus::FAILURE;
    }

    const auto objects = ctx_->world_model->getObjects();
    if (objects.empty()) {
        clear_frames_++;
        ctx_->wrench_adapter->setCommand(MotionCommand());
        ctx_->debug_msg = "CheckBottomClear: clear frame " +
                          std::to_string(clear_frames_) + "/" +
                          std::to_string(required_clear_frames);
        if (clear_frames_ >= required_clear_frames) {
            started_ = false;
            clear_frames_ = 0;
            ctx_->debug_msg = "CheckBottomClear: bottom view clear";
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    clear_frames_ = 0;

    double mean_cx = 0.0;
    for (const auto& obj : objects) {
        mean_cx += static_cast<double>(obj.cx);
    }
    mean_cx /= static_cast<double>(objects.size());

    MotionCommand cmd;
    const double offset_x = mean_cx - center_x;
    if (std::abs(offset_x) <= center_deadband) {
        cmd.sway = static_cast<float>(sway_speed);
    } else if (offset_x > 0.0) {
        cmd.sway = static_cast<float>(-sway_speed);
    } else {
        cmd.sway = static_cast<float>(sway_speed);
    }

    ctx_->wrench_adapter->setCommand(cmd);
    ctx_->debug_msg = "CheckBottomClear: swaying to clear " +
                      std::to_string(objects.size()) + " detection(s)";
    return BT::NodeStatus::RUNNING;
}

void CheckBottomClear::halt() {
    started_ = false;
    clear_frames_ = 0;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
