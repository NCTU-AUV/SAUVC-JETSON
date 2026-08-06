#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>
#include <set>

AvoidObstacle::AvoidObstacle(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList AvoidObstacle::providedPorts() {
    return { BT::InputPort<double>("sticky_time") };
}

BT::NodeStatus AvoidObstacle::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    double sticky_time = 1.0;
    getInput<double>("sticky_time", sticky_time);

    ctx_->current_action = name();
    ctx_->is_recovering = is_sticky_;

    auto objs = ctx_->world_model->getObjects();

    // Single source of truth for the detection image centre — see decision_node's
    // declaration. Do not reintroduce a bare literal here.
    const double centre_x = ctx_->node->get_parameter("image_center_x").as_double();

    std::set<std::string> flares = {"orange_flare", "red_flare", "blue_flare", "yellow_flare"};

    bool danger = false;
    TrackedObject danger_obj;

    for (const auto& obj : objs) {
        if (flares.count(obj.label)) {
            // Close enough to hit, and roughly ahead of us rather than off to
            // the side. 200 px of a 640-wide tensor is about the middle 62%.
            if (obj.distance > 0 && obj.distance < 2.0 && std::abs(obj.cx - centre_x) < 200.0) {
                danger = true;
                danger_obj = obj;
                break;
            }
        }
    }

    rclcpp::Time now = ctx_->node->now();

    if (danger) {
        is_sticky_ = true;
        last_avoid_time_ = now;
        ctx_->is_recovering = true;
        ctx_->debug_msg = "Avoiding " + danger_obj.label;
        
        MotionCommand cmd;
        cmd.surge = 5.0f; // small forward
        // Sway away from the object: if it is right of centre, sway left
        // (negative); if left of centre, sway right (positive).
        if (danger_obj.cx > centre_x) {
            cmd.sway = -10.0f; 
        } else {
            cmd.sway = 10.0f;
        }
        ctx_->wrench_adapter->setCommand(cmd);
        return BT::NodeStatus::RUNNING;
    }

    if (is_sticky_) {
        auto elapsed = (now - last_avoid_time_).seconds();
        if (elapsed < sticky_time) {
            // Keep running same command
            return BT::NodeStatus::RUNNING;
        } else {
            is_sticky_ = false;
        }
    }

    ctx_->wrench_adapter->setCommand(MotionCommand());
    return BT::NodeStatus::FAILURE; // No danger
}

void AvoidObstacle::halt() {
    is_sticky_ = false;
    if (ctx_) {
        ctx_->is_recovering = false;
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
