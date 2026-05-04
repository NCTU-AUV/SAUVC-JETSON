#include "orca_decision/world_model.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>

WorldModel::WorldModel(rclcpp::Node* node, double velocity_decay, double max_vel, double timeout_sec)
    : node_(node), velocity_decay_(velocity_decay), max_velocity_clamp_(max_vel), perception_timeout_sec_(timeout_sec)
{
}

void WorldModel::updateFromPerception(const orca_interface::msg::PerceptionArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    rclcpp::Time now = node_->now();

    for (const auto& obj : msg->objects) {
        if (!obj.is_stable) continue;

        TrackedObject tracked;
        tracked.label = obj.label;
        tracked.confidence = obj.confidence;
        tracked.distance = obj.distance;
        tracked.cx = obj.cx;
        tracked.cy = obj.cy;
        tracked.last_seen = now;

        // Project position to world frame
        Eigen::Vector3d pos_camera(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z);
        
        // For AUV, usually the camera frame is X right, Y down, Z forward. 
        // We will just store it directly for now, or map it using AUV orientation.
        // Assuming obj.pose is already relative to the AUV body frame or we apply orientation:
        Eigen::Vector3d pos_world = position_ + (orientation_ * pos_camera);
        tracked.position_world = pos_world;

        // Update existing or add new
        auto it = std::find_if(tracked_objects_.begin(), tracked_objects_.end(),
            [&](const TrackedObject& t) { return t.label == obj.label; });

        if (it != tracked_objects_.end()) {
            *it = tracked;
        } else {
            tracked_objects_.push_back(tracked);
        }
    }

    removeStaleObjects();
}

void WorldModel::updateFromIMU(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    rclcpp::Time current_time = msg->header.stamp;

    if (!imu_initialized_) {
        last_imu_time_ = current_time;
        imu_initialized_ = true;
        // Assume start at 0,0,0 and yaw=0
        position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        return;
    }

    double dt = (current_time - last_imu_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
        last_imu_time_ = current_time;
        return;
    }

    // Update orientation
    orientation_ = Eigen::Quaterniond(
        msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    orientation_.normalize();

    // Rotate linear acceleration to world frame
    Eigen::Vector3d acc_body(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    
    Eigen::Vector3d acc_world = orientation_ * acc_body;

    // Remove gravity (assume standard ROS ENU, Z is up, so sensor reads +9.81 when stationary)
    acc_world.z() -= 9.81;

    // Integrate velocity
    velocity_ += acc_world * dt;

    // Velocity decay (friction/drag model)
    velocity_ *= velocity_decay_;

    // Max velocity clamp
    if (velocity_.norm() > max_velocity_clamp_) {
        velocity_ = velocity_.normalized() * max_velocity_clamp_;
    }

    // Integrate position
    position_ += velocity_ * dt;

    last_imu_time_ = current_time;
}

std::optional<TrackedObject> WorldModel::getBestObject(const std::string& label) {
    std::lock_guard<std::mutex> lock(mutex_);
    removeStaleObjects();

    std::optional<TrackedObject> best;
    float max_conf = -1.0f;

    for (const auto& obj : tracked_objects_) {
        if (obj.label == label && obj.confidence > max_conf) {
            best = obj;
            max_conf = obj.confidence;
        }
    }

    return best;
}

std::vector<TrackedObject> WorldModel::getObjects() {
    std::lock_guard<std::mutex> lock(mutex_);
    removeStaleObjects();
    return tracked_objects_;
}

Eigen::Vector3d WorldModel::getAUVPosition() {
    std::lock_guard<std::mutex> lock(mutex_);
    return position_;
}

double WorldModel::getYaw() {
    std::lock_guard<std::mutex> lock(mutex_);
    // Convert quaternion to Euler angles (yaw, pitch, roll)
    tf2::Quaternion q(orientation_.x(), orientation_.y(), orientation_.z(), orientation_.w());
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

Eigen::Vector3d WorldModel::getAUVVelocity() {
    std::lock_guard<std::mutex> lock(mutex_);
    return velocity_;
}

void WorldModel::removeStaleObjects() {
    rclcpp::Time now = node_->now();
    tracked_objects_.erase(
        std::remove_if(tracked_objects_.begin(), tracked_objects_.end(),
            [&](const TrackedObject& obj) {
                return (now - obj.last_seen).seconds() > perception_timeout_sec_;
            }),
        tracked_objects_.end());
}
