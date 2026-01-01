cmake_minimum_required(VERSION 3.10)
project(gripper_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(motion_sdk REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(gripper_hardware SHARED
  src/gripper_hardware.cpp
)

target_include_directories(gripper_hardware PUBLIC
  include
)


ament_target_dependencies(gripper_hardware
  rclcpp
  hardware_interface
  pluginlib
  motion_sdk
)

pluginlib_export_plugin_description_file(
  hardware_interface plugin.xml
)

install(
  TARGETS gripper_hardware
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
