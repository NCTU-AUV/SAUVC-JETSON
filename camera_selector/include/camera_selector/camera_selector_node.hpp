// Copyright 2024 NCTU-AUV
// SPDX-License-Identifier: Apache-2.0

#ifndef CAMERA_SELECTOR__CAMERA_SELECTOR_NODE_HPP_
#define CAMERA_SELECTOR__CAMERA_SELECTOR_NODE_HPP_

#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"

namespace camera_selector
{

class CameraSelectorNode : public rclcpp::Node
{
public:
  explicit CameraSelectorNode(const rclcpp::NodeOptions & options);

private:
  /// Callback for /orca/camera_mode
  void mode_callback(const std_msgs::msg::String::SharedPtr msg);

  /// Callback for /orca/D435i/color/image_raw
  void realsense_callback(sensor_msgs::msg::Image::UniquePtr msg);

  /// Callback for /orca/D435i/color/camera_info
  void realsense_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg);

  /// Callback for /orca/usb_cam/image_raw
  void usb_callback(sensor_msgs::msg::Image::UniquePtr msg);

  /// Callback for /orca/usb_cam/camera_info
  void usb_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // --- Subscriptions ---
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_realsense_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_realsense_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_usb_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_usb_info_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;

  // --- Publisher ---
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_selected_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_selected_info_;

  // --- State ---
  std::string current_mode_;
  std::mutex mode_mutex_;

  sensor_msgs::msg::CameraInfo::SharedPtr latest_realsense_info_;
  std::mutex realsense_info_mutex_;

  sensor_msgs::msg::CameraInfo::SharedPtr latest_usb_info_;
  std::mutex usb_info_mutex_;
};

}  // namespace camera_selector

#endif  // CAMERA_SELECTOR__CAMERA_SELECTOR_NODE_HPP_
