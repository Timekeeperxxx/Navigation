cmake_minimum_required(VERSION 3.8)
project(nav_terrain)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)

add_executable(nav_terrain_analysis src/nav_terrain_analysis.cpp)
ament_target_dependencies(
  nav_terrain_analysis
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(nav_save_terrain_map src/nav_save_terrain_map.cpp)
ament_target_dependencies(
  nav_save_terrain_map
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_srvs
  PCL
)

install(TARGETS
  nav_terrain_analysis
  nav_save_terrain_map
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
