cmake_minimum_required(VERSION 3.16.3)
project(moveit_studio_za6_base_config)

find_package(ament_cmake REQUIRED)
find_package(za6_moveit_config REQUIRED)
find_package(za6_tools REQUIRED)

# Generate static SRDFs for the ZA6 + each gripper
gen_za6_gripper_srdfs()

# Build static URDFs of grippers for MoveIt Studio web preview
foreach(gripper ${za6_tools_grippers})
  gen_za6_tool_urdf(${gripper} ${CMAKE_CURRENT_BINARY_DIR}/urdf/${gripper}.urdf)
endforeach()

install(
  DIRECTORY
    config
    description
    objectives
    ${CMAKE_CURRENT_BINARY_DIR}/urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
