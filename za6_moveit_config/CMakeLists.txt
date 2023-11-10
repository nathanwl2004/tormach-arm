cmake_minimum_required(VERSION 3.22)
project(za6_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Build static SRDF
xacro_add_xacro_file(config/za6.srdf.xacro config/za6.srdf)
# - add an abstract target to actually trigger the builds
add_custom_target(srdf_files ALL DEPENDS ${XACRO_OUTPUT_FILE})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(FILES ${XACRO_OUTPUT_FILE} DESTINATION share/${PROJECT_NAME}/config)

ament_package(CONFIG_EXTRAS "za6_moveit_config-extras.cmake")
