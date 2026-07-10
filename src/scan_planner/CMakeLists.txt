cmake_minimum_required(VERSION 3.8)
project(scan_planner)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-O3 -Wall -g)

option(BUILD_SCAN_PLANNER_NODE "Build the core scan_planner_node" ON)
option(BUILD_SCAN_PLANNER_AUX_NODES "Build auxiliary controller/simulator nodes" ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Bspline.msg
  msg/DataDisp.msg
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

if(BUILD_SCAN_PLANNER_NODE OR BUILD_SCAN_PLANNER_AUX_NODES)
  set(scan_planner_dependencies
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    plan_env
    path_searching
    bspline_opt
    traj_utils
  )

  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
endif()

if(BUILD_SCAN_PLANNER_NODE)
  add_executable(scan_planner_node
    src/scan_planner_node.cpp
    src/scan_replan_fsm.cpp
    src/planner_manager.cpp
  )
  target_compile_features(scan_planner_node PUBLIC cxx_std_14)
  target_include_directories(scan_planner_node PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(scan_planner_node ${scan_planner_dependencies})
  target_link_libraries(scan_planner_node "${cpp_typesupport_target}")

  install(TARGETS scan_planner_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_SCAN_PLANNER_AUX_NODES)
  add_executable(open_loop_controller src/open_loop_controller.cpp)
  target_compile_features(open_loop_controller PUBLIC cxx_std_14)
  target_include_directories(open_loop_controller PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(open_loop_controller ${scan_planner_dependencies})

  add_executable(closed_loop_controller src/closed_loop_controller.cpp)
  target_compile_features(closed_loop_controller PUBLIC cxx_std_14)
  target_include_directories(closed_loop_controller PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(closed_loop_controller ${scan_planner_dependencies})

  add_executable(go2_kinematic_sim src/go2_kinematic_sim.cpp)
  target_compile_features(go2_kinematic_sim PUBLIC cxx_std_14)
  target_include_directories(go2_kinematic_sim PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(go2_kinematic_sim ${scan_planner_dependencies})

  add_executable(go2_gait_publisher src/go2_gait_publisher.cpp)
  target_compile_features(go2_gait_publisher PUBLIC cxx_std_14)
  target_include_directories(go2_gait_publisher PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(go2_gait_publisher ${scan_planner_dependencies})

  target_link_libraries(open_loop_controller "${cpp_typesupport_target}")
  target_link_libraries(closed_loop_controller "${cpp_typesupport_target}")
  target_link_libraries(go2_kinematic_sim "${cpp_typesupport_target}")
  target_link_libraries(go2_gait_publisher "${cpp_typesupport_target}")

  install(TARGETS
    open_loop_controller
    closed_loop_controller
    go2_kinematic_sim
    go2_gait_publisher
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
