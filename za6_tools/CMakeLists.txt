cmake_minimum_required(VERSION 3.22)
project(za6_tools)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()
