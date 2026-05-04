#include "orca_decision/wrench_adapter.hpp"

WrenchControllerAdapter::WrenchControllerAdapter(float k_surge, float k_sway, float k_yaw, float alpha)
    : k_surge_(k_surge), k_sway_(k_sway), k_yaw_(k_yaw), alpha_(alpha)
{
}

void WrenchControllerAdapter::setCommand(const MotionCommand& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_cmd_ = cmd;
}

geometry_msgs::msg::Wrench WrenchControllerAdapter::getWrench() {
    std::lock_guard<std::mutex> lock(mutex_);

    // Apply low-pass filter
    current_filtered_cmd_.surge += alpha_ * (target_cmd_.surge - current_filtered_cmd_.surge);
    current_filtered_cmd_.sway  += alpha_ * (target_cmd_.sway - current_filtered_cmd_.sway);
    current_filtered_cmd_.heave += alpha_ * (target_cmd_.heave - current_filtered_cmd_.heave);
    current_filtered_cmd_.yaw   += alpha_ * (target_cmd_.yaw - current_filtered_cmd_.yaw);

    geometry_msgs::msg::Wrench msg;
    
    // Mapping:
    // fx = k_surge * surge
    // fy = k_sway * sway
    // fz = heave (for now, assume k_heave is 1.0 or user just said heave controls fz)
    // tx = 0, ty = 0
    // tz = k_yaw * yaw

    msg.force.x = k_surge_ * current_filtered_cmd_.surge;
    msg.force.y = k_sway_  * current_filtered_cmd_.sway;
    msg.force.z = current_filtered_cmd_.heave; // based on user prompt "heave control fz"
    
    msg.torque.x = 0.0;
    msg.torque.y = 0.0;
    msg.torque.z = k_yaw_ * current_filtered_cmd_.yaw;

    return msg;
}
