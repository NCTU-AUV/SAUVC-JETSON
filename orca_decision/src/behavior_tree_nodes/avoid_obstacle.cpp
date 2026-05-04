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
    
    std::set<std::string> flares = {"orange_flare", "red_flare", "blue_flare", "yellow_flare"};
    
    bool danger = false;
    TrackedObject danger_obj;

    for (const auto& obj : objs) {
        if (flares.count(obj.label)) {
            // Check if distance < threshold and near center
            // Assume image width 640, center is 320, near center is say within 100 pixels
            // Distance threshold say 1.5m
            if (obj.distance > 0 && obj.distance < 1.5 && std::abs(obj.cx - 320.0) < 100.0) {
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
        cmd.surge = 0.2f; // small forward
        // Sway away from object. If obj is to the right (cx > 320), sway left (negative).
        // If obj is to the left (cx < 320), sway right (positive).
        if (danger_obj.cx > 320.0f) {
            cmd.sway = -0.5f; 
        } else {
            cmd.sway = 0.5f;
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
