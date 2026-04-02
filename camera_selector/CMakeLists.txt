cmake_minimum_required(VERSION 3.8)
project(camera_selector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Find dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

# =========================
# Shared library (composable node)
# =========================
add_library(camera_selector_component SHARED
  src/camera_selector_node.cpp
)
target_include_directories(camera_selector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_selector_component ${DEPENDENCIES})

# Register composable node
rclcpp_components_register_node(camera_selector_component
  PLUGIN "camera_selector::CameraSelectorNode"
  EXECUTABLE camera_selector_node
)

# =========================
# Install
# =========================
install(TARGETS camera_selector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
