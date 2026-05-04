cmake_minimum_required(VERSION 3.8)
project(depth_perception)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(orca_interface REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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
