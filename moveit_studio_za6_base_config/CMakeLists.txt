cmake_minimum_required(VERSION 3.16.3)
project(moveit_studio_za6_base_config)

find_package(ament_cmake REQUIRED)
find_package(za6_moveit_config REQUIRED)

# Generate static SRDFs for the ZA6 + each gripper
gen_za6_gripper_srdfs()

install(
  DIRECTORY
    config
    objectives
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
