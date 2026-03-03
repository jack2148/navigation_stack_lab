
cmake_minimum_required(VERSION 3.8)
project(robot_base)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(robot_base_hw SHARED src/can_hw.cpp)
ament_target_dependencies(robot_base_hw rclcpp hardware_interface pluginlib)

pluginlib_export_plugin_description_file(hardware_interface robot_base_hw.xml)

install(TARGETS robot_base_hw LIBRARY DESTINATION lib)
install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})

ament_package()
