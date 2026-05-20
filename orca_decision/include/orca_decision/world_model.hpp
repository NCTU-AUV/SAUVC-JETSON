#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <orca_interface/msg/perception_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <string>
#include <vector>
#include <optional>
#include <unordered_map>

struct TrackedObject {
    std::string label;
    float confidence;
    Eigen::Vector3d position_world; // Position in world frame
    float distance; // Distance from AUV
    float cx;       // Camera frame cx
    float cy;       // Camera frame cy
    rclcpp::Time last_seen;
};

class WorldModel {
public:
    WorldModel(rclcpp::Node* node, double velocity_decay, double max_vel, double timeout_sec);

    void updateFromPerception(const orca_interface::msg::PerceptionArray::SharedPtr msg);
    void updateFromIMU(const sensor_msgs::msg::Imu::SharedPtr msg);

    std::optional<TrackedObject> getBestObject(const std::string& label);
    std::optional<TrackedObject> getObjectNearestImageCenter(
        const std::string& label, float center_x, float center_y);
    std::vector<TrackedObject> getObjects();

    Eigen::Vector3d getAUVPosition();
    double getYaw();
    Eigen::Vector3d getAUVVelocity();

private:
    rclcpp::Node* node_;
    std::mutex mutex_;

    // Config parameters
    double velocity_decay_;
    double max_velocity_clamp_;
    double perception_timeout_sec_;

    // AUV State
    Eigen::Vector3d position_{0.0, 0.0, 0.0};
    Eigen::Vector3d velocity_{0.0, 0.0, 0.0};
    Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};
    rclcpp::Time last_imu_time_;
    bool imu_initialized_ = false;

    // Tracked objects
    // Key: label (We can have multiple objects of same label, but we track the most recent/best ones.
    // For simplicity, we just keep a list and cull old ones.)
    std::vector<TrackedObject> tracked_objects_;

    void removeStaleObjects();
};
