cmake_minimum_required(VERSION 3.22)
project(za6_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf test
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  set(grippers pivot_gripper robotiq_hand_e versabuilt_multigrip none "")
  foreach(gripper ${grippers})
    ament_add_pytest_test(
      za6_${gripper}_urdf_xacro test/test_za6_urdf_xacro.py
      ENV ZA_GRIPPER=${gripper}
    )
    ament_add_pytest_test(
      view_za6_${gripper}_launch test/test_view_za6_launch.py
      ENV ZA_GRIPPER=${gripper}
    )
  endforeach()
endif()

ament_package()
