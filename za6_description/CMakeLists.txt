cmake_minimum_required(VERSION 3.22)
project(za6_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf test
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(za6_urdf_xacro test/test_za6_urdf_xacro.py)
  ament_add_pytest_test(view_za6_launch test/test_view_za6_launch.py)
endif()

ament_package()
