cmake_minimum_required(VERSION 3.22)
project(za6_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Build static SRDFs for each gripper
set(grippers pivot_gripper robotiq_hand_e versabuilt_multigrip none)

foreach(gripper ${grippers})
  set(output_filename config/za6.${gripper}.srdf)

  # create a rule to generate ${output_filename} from the SRDF
  xacro_add_xacro_file(
    config/za6.srdf.xacro ${output_filename}
    REMAP gripper:=${gripper})

  list(APPEND za6_srdf_files ${XACRO_OUTPUT_FILE})
endforeach(gripper)

# Build static SRDF for *no* gripper
xacro_add_xacro_file(config/za6.srdf.xacro config/za6.srdf)
list(APPEND za6_srdf_files ${XACRO_OUTPUT_FILE})

# add an abstract target to actually trigger the builds
add_custom_target(srdf_files ALL DEPENDS ${za6_srdf_files})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(FILES ${za6_srdf_files} DESTINATION share/${PROJECT_NAME}/config)


ament_package()
