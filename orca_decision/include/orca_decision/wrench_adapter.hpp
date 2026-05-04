#pragma once

#include <geometry_msgs/msg/wrench.hpp>
#include <mutex>

struct MotionCommand {
    float surge = 0.0f;
    float sway = 0.0f;
    float heave = 0.0f;
    float yaw = 0.0f;
};

class WrenchControllerAdapter {
public:
    WrenchControllerAdapter(float k_surge, float k_sway, float k_yaw, float alpha);

    void setCommand(const MotionCommand& cmd);
    geometry_msgs::msg::Wrench getWrench();

private:
    std::mutex mutex_;
    
    // Gains
    float k_surge_;
    float k_sway_;
    float k_yaw_;

    // Low-pass filter alpha
    float alpha_;

    // State
    MotionCommand current_filtered_cmd_;
    MotionCommand target_cmd_;
};
