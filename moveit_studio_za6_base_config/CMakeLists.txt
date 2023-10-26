cmake_minimum_required(VERSION 3.16.3)
project(moveit_studio_za6_base_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    objectives
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
