cmake_minimum_required(VERSION 3.22)
project(za6_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()
