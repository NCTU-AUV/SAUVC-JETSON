cmake_minimum_required(VERSION 3.8)
project(fsm_decision)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# install(
#   DIRECTORY fsm_decision
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   PROGRAMS
#     fsm_decision/fsm_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
