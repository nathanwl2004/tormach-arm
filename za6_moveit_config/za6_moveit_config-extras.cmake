# find_package(za6_moveit_config REQUIRED)

find_package(xacro REQUIRED)
find_package(za6_tools REQUIRED)

# SRDF xacro file
set(za6_srdf_xacro ${za6_moveit_config_DIR}/../config/za6.srdf.xacro)

# Call gen_za6_gripper_srdfs("robot_base_mount") to generate SRDFs for
# the ZA6, one for each gripper, with ZA6 base and robot_base_mount
# collisions disabled.  With no arg, don't set robot_base_mount
# collisions.
function(gen_za6_gripper_srdfs)
  list(LENGTH ARGN extra_count)
  if(${extra_count} GREATER 1)
    message(FATAL_ERROR "Too many arguments to gen_za6_gripper_srdfs()")
  endif()
  set(robot_base_link "${ARGN}")

  foreach(gripper ${za6_tools_grippers})
    set(output_filename config/za6.${gripper}.srdf)

    # create a rule to generate ${output_filename} from the SRDF
    xacro_add_xacro_file(
      ${za6_srdf_xacro} ${output_filename} REMAP gripper:=${gripper}
      mount_link:=${robot_base_link}
      )

    list(APPEND za6_srdf_files ${XACRO_OUTPUT_FILE})
  endforeach()

  # add an abstract target to actually trigger the builds
  add_custom_target(srdf_files ALL DEPENDS ${za6_srdf_files})

  install(FILES ${za6_srdf_files} DESTINATION share/${PROJECT_NAME}/config)
endfunction()
