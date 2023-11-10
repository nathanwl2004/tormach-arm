# Copyright (c) 2023 Tormach, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
