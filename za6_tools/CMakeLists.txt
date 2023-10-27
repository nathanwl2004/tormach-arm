cmake_minimum_required(VERSION 3.22)
project(za6_tools)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

set(grippers pivot_gripper robotiq_hand_e versabuilt_multigrip)

# Build static URDFs of stand-alone grippers for MoveIt Studio web preview
set(standalone_gripper_urdf_files "${grippers}")
list(TRANSFORM standalone_gripper_urdf_files PREPEND urdf/)
list(TRANSFORM standalone_gripper_urdf_files APPEND .urdf.xacro)
xacro_add_files(${standalone_gripper_urdf_files}
  TARGET gripper_urdf_files INSTALL DESTINATION urdf)



install(DIRECTORY config meshes urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
