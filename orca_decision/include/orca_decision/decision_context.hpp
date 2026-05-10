#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "orca_decision/world_model.hpp"
#include "orca_decision/wrench_adapter.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

struct DecisionContext {
    rclcpp::Node* node;
    std::shared_ptr<WorldModel> world_model;
    std::shared_ptr<WrenchControllerAdapter> wrench_adapter;
    
    // Some common state flags
    bool mission_started = false;
    rclcpp::Time mission_start_time;
    std::string current_flare_order = "";

    // State for DecisionStatus
    std::string current_action = "";
    std::string target_label = "";
    bool is_recovering = false;
    std::string camera_mode = "";
    std::string debug_msg = "";

    // Thresholds
    float align_yaw_threshold = 0.1f;
    float align_distance_threshold = 0.5f;

    // Publishers needed by BT nodes
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_mode_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr desired_depth_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr hand_pub;
};
