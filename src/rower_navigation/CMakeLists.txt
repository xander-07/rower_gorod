cmake_minimum_required(VERSION 3.8)
project(rower_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config web
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/scan_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
