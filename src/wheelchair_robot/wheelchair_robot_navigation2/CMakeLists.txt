cmake_minimum_required(VERSION 3.5)
project(wheelchair_robot_navigation2)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch map param rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
