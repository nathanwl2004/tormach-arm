cmake_minimum_required(VERSION 3.22)
project(za6_hardware)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Find Machinekit HAL
find_package(hal_hw_interface REQUIRED)
find_package(HAL REQUIRED)
include(UseHALComp)

#
# Build
#

ament_python_install_package(hal_plumber)

hal_add_instcomp(src/limit3v3)
hal_add_instcomp(src/ppi)
hal_add_instcomp(src/pll)
hal_add_instcomp(src/est)
hal_add_instcomp(src/latency)
hal_add_instcomp(src/qc)
hal_add_instcomp(src/drive_safety)
#
# Tests
#

if(BUILD_TESTING)

  # find_package(ament_cmake_flake8 REQUIRED)
  # ament_flake8()
  # find_package(ament_cmake_pep257 REQUIRED)
  # ament_pep257()
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

#
# Install
#

# Install python executables
install(
  PROGRAMS scripts/hw_device_mgr scripts/drive_state scripts/dump_params
  DESTINATION DESTINATION
  lib/${PROJECT_NAME} # or bin?
)

# Install other files
install(DIRECTORY config halfiles launch DESTINATION share/${PROJECT_NAME})

#
# Finalize package
#

ament_package()
