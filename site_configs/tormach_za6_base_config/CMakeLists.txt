cmake_minimum_required(VERSION 3.16.3)
project(tormach_za6_base_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    objectives
    rviz
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
