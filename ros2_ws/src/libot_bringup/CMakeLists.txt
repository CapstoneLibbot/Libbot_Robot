cmake_minimum_required(VERSION 3.8)
project(libot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/manual_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
