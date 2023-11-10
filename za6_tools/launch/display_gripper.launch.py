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

import os
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def create_robot_description(context):
    xacro_file = os.path.join(
        get_package_share_directory("za6_tools"),
        "urdf",
        "gripper.urdf.xacro",
    )
    assert os.path.exists(xacro_file), f"No such file, '{xacro_file}'"
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings=dict(
            prefix=context.launch_configurations["prefix"],
            gripper=context.launch_configurations["gripper"],
        )
    )
    robot_desc = robot_description_config.toxml()
    return [SetLaunchConfiguration("robot_desc", robot_desc)]

def generate_launch_description():

    gripper = DeclareLaunchArgument(
        "gripper",
        default_value="pivot_gripper",
        description="Gripper to display",
    )

    prefix = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description="Namespace prefix",
    )

    use_jspg = DeclareLaunchArgument(
        "use_joint_state_pub_gui",
        default_value="true",
        description="Start joint_state_publisher_gui node",
    )


    robot_description_config = OpaqueFunction(function=create_robot_description)
    robot_description = dict(
        robot_description = ParameterValue(
            LaunchConfiguration('robot_desc'), value_type=str
        )
    )

    # joint_state_pub_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher'
    # )

    joint_state_pub_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration("use_joint_state_pub_gui")),
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='log',
      arguments=[
          '-d',
          PathJoinSubstitution(
              [
                  get_package_share_directory("za6_tools"),
                  "rviz",
                  PythonExpression(
                      expression=[
                          "'", LaunchConfiguration("gripper"), "' + '.rviz'"
                      ]
                  ),
              ],
          ),
      ],
      parameters=[robot_description]
      )

    return LaunchDescription(
        [
            gripper,
            prefix,
            use_jspg,
            robot_description_config,
            rviz_node,
            robot_state_pub_node,
            joint_state_pub_gui_node,
        ]
    )
