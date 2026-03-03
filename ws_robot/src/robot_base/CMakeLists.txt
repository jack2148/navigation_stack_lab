cmake_minimum_required(VERSION 3.8)
project(robot_base)

# Optional but recommended
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ROS2 / ros2_control deps
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(robot_base_hw SHARED
  src/can_hw.cpp
)

ament_target_dependencies(robot_base_hw
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface robot_base_hw.xml)

# Install library & resources
install(TARGETS robot_base_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()