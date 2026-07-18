cmake_minimum_required(VERSION 3.8)
project(nav_bringup)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(
  livox_lidar_timestamp_monitor
  src/livox_lidar_timestamp_monitor.cpp
)
ament_target_dependencies(
  livox_lidar_timestamp_monitor
  livox_ros_driver2
  rclcpp
)

add_executable(
  nav_pcd_map_publisher
  src/nav_pcd_map_publisher.cpp
)
target_include_directories(nav_pcd_map_publisher PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(nav_pcd_map_publisher ${PCL_LIBRARIES})
ament_target_dependencies(
  nav_pcd_map_publisher
  rclcpp
  sensor_msgs
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pcd_debug_publisher.py
  scripts/rviz_2d_goal_planner.py
  scripts/scan_initial_path_adapter.py
  scripts/scan_tf_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  livox_lidar_timestamp_monitor
  nav_pcd_map_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
