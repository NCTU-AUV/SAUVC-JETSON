// Copyright 2024 NCTU-AUV
// SPDX-License-Identifier: Apache-2.0

#include "camera_selector/camera_selector_node.hpp"

#include <functional>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

namespace camera_selector
{

CameraSelectorNode::CameraSelectorNode(const rclcpp::NodeOptions & options)
: Node("camera_selector_node", options)
{
  // ── Declare parameters (values come from YAML / ROS param server) ──
  this->declare_parameter<std::string>("realsense_image_topic", "/orca/color/image_raw");
  this->declare_parameter<std::string>("realsense_info_topic",  "/orca/color/camera_info");
  this->declare_parameter<std::string>("usb_image_topic",       "/orca/usb_cam/image_raw");
  this->declare_parameter<std::string>("usb_info_topic",        "/orca/usb_cam/camera_info");
  // Must match what decision_node publishes; a stale default here silently
  // pins the selector to whatever default_mode says for the whole run.
  this->declare_parameter<std::string>("mode_topic",            "/orca/decision/camera_mode");
  this->declare_parameter<std::string>("selected_image_topic",  "/orca/selected/image_raw");
  this->declare_parameter<std::string>("selected_info_topic",   "/orca/selected/camera_info");
  this->declare_parameter<std::string>("default_mode",          "realsense");

  const auto realsense_image_topic = this->get_parameter("realsense_image_topic").as_string();
  const auto realsense_info_topic  = this->get_parameter("realsense_info_topic").as_string();
  const auto usb_image_topic       = this->get_parameter("usb_image_topic").as_string();
  const auto usb_info_topic        = this->get_parameter("usb_info_topic").as_string();
  const auto mode_topic            = this->get_parameter("mode_topic").as_string();
  const auto selected_image_topic  = this->get_parameter("selected_image_topic").as_string();
  const auto selected_info_topic   = this->get_parameter("selected_info_topic").as_string();
  current_mode_                    = this->get_parameter("default_mode").as_string();

  // ── QoS profiles ──────────────────────────────────────────────
  // Sensor data: best-effort, volatile, keep-last(1)
  auto sensor_qos = rclcpp::SensorDataQoS();

  // Mode topic: reliable, keep-last(1), transient-local so late
  // subscribers still get the last published mode
  auto mode_qos = rclcpp::QoS(1)
    .reliable()
    .transient_local();

  // Publisher QoS: match the standard ROS 2 reliable QoS expected by Isaac ROS
  auto pub_qos = rclcpp::QoS(10).reliable();

  // ── Subscriptions ─────────────────────────────────────────────
  sub_realsense_ = this->create_subscription<sensor_msgs::msg::Image>(
    realsense_image_topic,
    sensor_qos,
    std::bind(&CameraSelectorNode::realsense_callback, this, std::placeholders::_1));

  sub_realsense_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    realsense_info_topic,
    sensor_qos,
    std::bind(&CameraSelectorNode::realsense_info_callback, this, std::placeholders::_1));

  sub_usb_ = this->create_subscription<sensor_msgs::msg::Image>(
    usb_image_topic,
    sensor_qos,
    std::bind(&CameraSelectorNode::usb_callback, this, std::placeholders::_1));

  sub_usb_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    usb_info_topic,
    sensor_qos,
    std::bind(&CameraSelectorNode::usb_info_callback, this, std::placeholders::_1));

  sub_mode_ = this->create_subscription<std_msgs::msg::String>(
    mode_topic,
    mode_qos,
    std::bind(&CameraSelectorNode::mode_callback, this, std::placeholders::_1));

  // ── Publisher ─────────────────────────────────────────────────
  pub_selected_ = this->create_publisher<sensor_msgs::msg::Image>(
    selected_image_topic,
    pub_qos);

  pub_selected_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    selected_info_topic,
    pub_qos);

  RCLCPP_INFO(this->get_logger(),
    "Camera selector initialised — default mode: '%s'", current_mode_.c_str());
}

// ─────────────────────────────────────────────────────────────────
void CameraSelectorNode::mode_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  const auto & mode = msg->data;
  if (mode != "realsense" && mode != "usb") {
    RCLCPP_WARN(this->get_logger(),
      "Unknown camera mode '%s' — ignoring (valid: realsense, usb)",
      mode.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    if (current_mode_ != mode) {
      RCLCPP_INFO(this->get_logger(),
        "Camera mode switched: '%s' -> '%s'",
        current_mode_.c_str(), mode.c_str());
      current_mode_ = mode;
    }
  }
}

// ─────────────────────────────────────────────────────────────────
void CameraSelectorNode::realsense_info_callback(
  sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(realsense_info_mutex_);
  latest_realsense_info_ = msg;
}

// ─────────────────────────────────────────────────────────────────
void CameraSelectorNode::usb_info_callback(
  sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(usb_info_mutex_);
  latest_usb_info_ = msg;
}

// ─────────────────────────────────────────────────────────────────
void CameraSelectorNode::realsense_callback(
  sensor_msgs::msg::Image::UniquePtr msg)
{
  std::string mode;
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    mode = current_mode_;
  }

  if (mode == "realsense") {
    sensor_msgs::msg::CameraInfo::SharedPtr cached_info;
    {
      std::lock_guard<std::mutex> lock(realsense_info_mutex_);
      cached_info = latest_realsense_info_;
    }

    if (!cached_info) {
      return;
    }

    sensor_msgs::msg::CameraInfo synced_info = *cached_info;
    synced_info.header.stamp = msg->header.stamp;

    pub_selected_info_->publish(synced_info);
    pub_selected_->publish(std::move(msg));
  }
}

// ─────────────────────────────────────────────────────────────────
void CameraSelectorNode::usb_callback(
  sensor_msgs::msg::Image::UniquePtr msg)
{
  std::string mode;
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    mode = current_mode_;
  }

  if (mode == "usb") {
    sensor_msgs::msg::CameraInfo::SharedPtr cached_info;
    {
      std::lock_guard<std::mutex> lock(usb_info_mutex_);
      cached_info = latest_usb_info_;
    }

    if (!cached_info) {
      return;
    }

    sensor_msgs::msg::CameraInfo synced_info = *cached_info;
    synced_info.header.stamp = msg->header.stamp;

    pub_selected_info_->publish(synced_info);
    pub_selected_->publish(std::move(msg));
  }
}

}  // namespace camera_selector

// Register as composable node component
RCLCPP_COMPONENTS_REGISTER_NODE(camera_selector::CameraSelectorNode)
