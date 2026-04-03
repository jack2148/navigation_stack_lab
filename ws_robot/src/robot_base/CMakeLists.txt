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
# IMU용 메시지 패키지 추가
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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

# --- IMU 노드 실행파일 추가 ---
add_executable(imu_node src/imu.cpp)
ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install library & resources
install(TARGETS robot_base_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- IMU 노드 설치 경로 추가 ---
install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- 파이썬 스크립트 설치 경로 ---
install(PROGRAMS
  src/patrol_node.py
  src/robot_pose_tf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()