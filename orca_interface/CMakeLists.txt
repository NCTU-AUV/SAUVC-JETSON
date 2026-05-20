cmake_minimum_required(VERSION 3.8)
project(orca_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Find dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =====================
# Interface generation
# =====================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerceptionObject.msg"
  "msg/PerceptionArray.msg"
  "msg/DecisionStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# =====================
# Export
# =====================
ament_export_dependencies(rosidl_default_runtime)

ament_package()
