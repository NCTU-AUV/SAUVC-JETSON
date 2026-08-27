#include "orca_decision/behavior_tree_nodes.hpp"
#include <cmath>

SearchTarget::SearchTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList SearchTarget::providedPorts() {
    return { 
        BT::InputPort<std::string>("label"),
        BT::InputPort<double>("yaw_speed", 0.5, "Yaw sweeping speed in rad/s (default: 0.5)"),
        BT::InputPort<double>("center_threshold", 120.0, "Error threshold in pixels to consider target centered"),
        BT::InputPort<int>("stable_frames", 2, "Number of frames target must be centered to return SUCCESS")
    };
}

BT::NodeStatus SearchTarget::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    std::string label;
    if (!getInput<std::string>("label", label)) {
        throw BT::RuntimeError("missing required input [label]");
    }

    double yaw_speed = 0.5;
    getInput<double>("yaw_speed", yaw_speed);

    double center_threshold = 120.0;
    getInput<double>("center_threshold", center_threshold);

    int required_stable_frames = 2;
    getInput<int>("stable_frames", required_stable_frames);

    ctx_->current_action = name();
    ctx_->target_label = label;
    ctx_->debug_msg = "Searching target: " + label;

    auto obj = ctx_->world_model->getBestObject(label);
    if (obj.has_value()) {
        const float centre_x =
            static_cast<float>(ctx_->node->get_parameter("image_center_x").as_double());
        
        float error_x = centre_x - obj->cx;
        
        if (std::abs(error_x) < center_threshold) {
            stable_frames_++;
            if (stable_frames_ >= required_stable_frames) {
                ctx_->wrench_adapter->setCommand(MotionCommand()); // stop
                return BT::NodeStatus::SUCCESS;
            }
        } else {
            stable_frames_ = 0;
        }

        // Target found but not centered, apply yaw P-control
        MotionCommand cmd;
        cmd.yaw = std::clamp(-0.0035f * error_x, -0.35f, 0.35f);
        cmd.surge = 0.0f; // Ensure surge is zero while aligning
        ctx_->wrench_adapter->setCommand(cmd);

        // Update sweep direction based on where we last saw it
        // If error_x < 0 (target is on the right), we need to yaw positive (starboard)
        sweep_direction_ = (error_x < 0) ? 1 : -1; 

        return BT::NodeStatus::RUNNING;
    }

    // Not found, perform yaw sweep
    stable_frames_ = 0;
    MotionCommand cmd;
    cmd.yaw = static_cast<float>(yaw_speed * sweep_direction_);
    ctx_->wrench_adapter->setCommand(cmd);

    return BT::NodeStatus::RUNNING;
}

void SearchTarget::halt() {
    stable_frames_ = 0;
    if (ctx_) {
        ctx_->wrench_adapter->setCommand(MotionCommand());
    }
}
