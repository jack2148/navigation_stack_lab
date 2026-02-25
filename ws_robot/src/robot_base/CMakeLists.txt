cmake_minimum_required(VERSION 3.8)
project(robot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderTicks.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

add_executable(can_bridge_node src/can_bridge.cpp)
ament_target_dependencies(can_bridge_node
  rclcpp std_msgs
)
# Link message typesupport to executables that use generated messages
rosidl_target_interfaces(can_bridge_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(wheel_odometry_node src/wheel_odometry.cpp)
ament_target_dependencies(wheel_odometry_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)
rosidl_target_interfaces(wheel_odometry_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  can_bridge_node
  wheel_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Export message runtime dependency
ament_export_dependencies(rosidl_default_runtime)

ament_package()
