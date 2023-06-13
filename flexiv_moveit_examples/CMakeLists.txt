cmake_minimum_required(VERSION 3.5)
project(flexiv_moveit_examples)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  moveit_core
  moveit_ros_planning_interface
)

install(
  DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(move_group_waypoints
  src/move_group_waypoints.cpp)
target_include_directories(move_group_waypoints
  PUBLIC include)
ament_target_dependencies(move_group_waypoints
  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS move_group_waypoints
  DESTINATION lib/${PROJECT_NAME})

ament_package()
