#pragma once

#include <geometry_msgs/msg/wrench.hpp>
#include <orca_interface/msg/decision_status.hpp>
#include <orca_interface/msg/perception_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>

#include "orca_decision/decision_context.hpp"
#include "orca_decision/world_model.hpp"
#include "orca_decision/wrench_adapter.hpp"

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode();

private:
  void loadParameters();
  void setupInterfaces();
  void registerBehaviorTree();
  void btTickLoop();
  void publishWrenchLoop();

  // Callbacks
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void
  perceptionCallback(const orca_interface::msg::PerceptionArray::SharedPtr msg);
  void startMissionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void flareOrderCallback(const std_msgs::msg::String::SharedPtr msg);

  // Context & Logic
  std::shared_ptr<DecisionContext> ctx_;

  // BT
  BT::BehaviorTreeFactory bt_factory_;
  std::unique_ptr<BT::Tree> tree_;

  // ROS Timers
  rclcpp::TimerBase::SharedPtr bt_timer_;
  rclcpp::TimerBase::SharedPtr wrench_timer_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<orca_interface::msg::PerceptionArray>::SharedPtr
      perception_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flare_order_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr desired_depth_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hand_pub_;
  rclcpp::Publisher<orca_interface::msg::DecisionStatus>::SharedPtr status_pub_;

  std::string tree_xml_file_;
  std::string main_tree_id_;
  bool mission_complete_ = false;
};
