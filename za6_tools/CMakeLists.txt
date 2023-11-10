cmake_minimum_required(VERSION 3.22)
project(za6_tools)

find_package(ament_cmake REQUIRED)

# The authoritative list of grippers, used by other packages
set(za6_tools_grippers pivot_gripper robotiq_hand_e versabuilt_multigrip none)

configure_file(cmake/za6_tools_grippers.cmake.in za6_tools_grippers.cmake @ONLY)

install(DIRECTORY config launch meshes rviz test urdf
  DESTINATION share/${PROJECT_NAME})
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/za6_tools_grippers.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

if(BUILD_TESTING)

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()

  find_package(ament_cmake_pytest REQUIRED)

  # Grippers
  foreach(gripper ${za6_tools_grippers})
    ament_add_pytest_test(
      ${gripper}_urdf_xacro test/test_gripper_xacro.py
      ENV ZA_GRIPPER=${gripper}
    )
    ament_add_pytest_test(
      display_${gripper}_launch test/test_display_gripper_launch.py
      ENV ZA_GRIPPER=${gripper}
    )
  endforeach()

  # PCNC1100
  ament_add_pytest_test(
    pcnc1100_urdf_xacro test/test_pcnc1100_xacro.py
  )
  ament_add_pytest_test(
    display_pcnc1100_launch test/test_display_pcnc1100_launch.py
  )
endif()

ament_package(
  CONFIG_EXTRAS "za6_tools-extras.cmake"
)
