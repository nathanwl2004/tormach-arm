cmake_minimum_required(VERSION 3.22)
project(moveit_studio_za6_tending_config)

find_package(ament_cmake REQUIRED)
find_package(za6_moveit_config REQUIRED)

gen_za6_gripper_srdfs(robot_base)

install(
  DIRECTORY
    config
    description
    meshes
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
