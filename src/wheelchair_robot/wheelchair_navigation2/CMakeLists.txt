cmake_minimum_required(VERSION 3.5)
project(wheelchair_navigation2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_library(is_threat_present_condition_bt_node SHARED
  src/is_threat_present_condition_bt_node.cpp
)

target_include_directories(is_threat_present_condition_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(is_threat_present_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(is_threat_present_condition_bt_node
  rclcpp
  sensor_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
)

install(TARGETS is_threat_present_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch map param rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
