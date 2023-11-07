cmake_minimum_required(VERSION 3.22)
project(za6_tools)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

set(grippers pivot_gripper robotiq_hand_e versabuilt_multigrip)

# Build static URDFs of grippers for MoveIt Studio web preview
set(gripper_urdf_files "${grippers}")
list(TRANSFORM gripper_urdf_files PREPEND urdf/)
list(TRANSFORM gripper_urdf_files APPEND .urdf.xacro)
xacro_add_files(${gripper_urdf_files}
  REMAP build:=true
  TARGET gripper_urdf_files
  INSTALL DESTINATION urdf)



install(DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
