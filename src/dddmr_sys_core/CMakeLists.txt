cmake_minimum_required(VERSION 3.5)
project(dddmr_sys_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  builtin_interfaces
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PToPMoveBase.action"
  "action/GetPlan.action"
  "action/RecoveryBehaviors.action"
  "action/TagDocking.action"
  "srv/GetKeyFrameCloud.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs sensor_msgs
)

include_directories(
  include
)

# dddmr_sys_core
add_library(dddmr_sys_core_enum SHARED
  src/dddmr_enum_states.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
dddmr_sys_core rosidl_typesupport_cpp)

target_link_libraries(dddmr_sys_core_enum "${cpp_typesupport_target}")

ament_target_dependencies(dddmr_sys_core_enum
  ${dependencies}
)

install(TARGETS
  dddmr_sys_core_enum
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(dddmr_sys_core_enum)
ament_export_dependencies(${dependencies})
ament_export_dependencies(rosidl_default_runtime)

ament_package()
