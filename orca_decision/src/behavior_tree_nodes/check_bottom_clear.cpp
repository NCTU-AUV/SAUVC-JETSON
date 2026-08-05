#include "orca_decision/behavior_tree_nodes.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

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
    const double surge_speed =
        ctx_->node->get_parameter("check_bottom_clear_surge_speed").as_double();
    const int required_clear_frames =
        ctx_->node->get_parameter("check_bottom_clear_stable_frames").as_int();

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

    // "Clear" means nothing is still under us — not "the camera sees nothing
    // anywhere". getObjects() returns every tracked object regardless of label
    // or image position, so the previous `objects.empty()` test treated any
    // detection at the edge of frame, of any class, as a blocker: the drum we
    // just dropped on is still in the world model (stability_window keeps it),
    // so the node reversed at 15.0 surge on its very first tick, every time.
    //
    // check_bottom_clear_center_deadband was declared and configured for
    // exactly this and then never read. Use it: only objects within the
    // deadband of the image centre count.
    const double deadband =
        ctx_->node->get_parameter("check_bottom_clear_center_deadband").as_double();
    const double centre_x = ctx_->node->get_parameter("image_center_x").as_double();
    const double centre_y = ctx_->node->get_parameter("image_center_y").as_double();

    std::vector<TrackedObject> blocking;
    for (const auto& obj : ctx_->world_model->getObjects()) {
        const double dx = obj.cx - centre_x;
        const double dy = obj.cy - centre_y;
        if (std::hypot(dx, dy) <= deadband) {
            blocking.push_back(obj);
        }
    }

    const auto& objects = blocking;
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

    MotionCommand cmd;
    cmd.surge = static_cast<float>(-surge_speed);

    ctx_->wrench_adapter->setCommand(cmd);
    ctx_->debug_msg = "CheckBottomClear: backing up to clear " +
                      std::to_string(objects.size()) +
                      " detection(s) under the vehicle";
    return BT::NodeStatus::RUNNING;
}

void CheckBottomClear::halt() {
    started_ = false;
    clear_frames_ = 0;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
