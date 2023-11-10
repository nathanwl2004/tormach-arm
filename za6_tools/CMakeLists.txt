cmake_minimum_required(VERSION 3.22)
project(za6_tools)

find_package(ament_cmake REQUIRED)

# The authoritative list of grippers, used by other packages
set(grippers pivot_gripper robotiq_hand_e versabuilt_multigrip none)

configure_file(cmake/za6_tools_grippers.cmake.in za6_tools_grippers.cmake @ONLY)

install(DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME})
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/za6_tools_grippers.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_package(
  CONFIG_EXTRAS "za6_tools-extras.cmake"
)
