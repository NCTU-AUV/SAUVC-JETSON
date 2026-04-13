cmake_minimum_required(VERSION 3.8)
project(depth_perception)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# =========================
# Message generation
# =========================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerceptionObject.msg"
  "msg/PerceptionArray.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# =========================
# Install Python nodes
# =========================
install(PROGRAMS
  scripts/depth_perception_node.py
  scripts/depth_perception_viz_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Install launch files
# =========================
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
