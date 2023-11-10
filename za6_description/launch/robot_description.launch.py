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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="za6_description",
            description="Description package with robot URDF/XACRO files",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="za6.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description=(
                "Prefix of joint names for multi-robot setup; "
                "if changed, controller configuration joint names "
                "must also be updated"
            ),
        ),
        DeclareLaunchArgument(
            "include_ros2_control",
            default_value="true",
            description=("If true, include the HAL hardware ros2_control URDF"),
        ),
        DeclareLaunchArgument(
            "ros2_control_name",
            default_value="hal_hw_interface",
            description=("ros2_control hardware name"),
        ),
        DeclareLaunchArgument(
            "ros2_control_plugin",
            default_value="hal_system_interface/HalSystemInterface",
            description=(
                "ros2_control plugin; default HAL hw interface;"
                " fake hardware use `mock_components/GenericSystem`"
            ),
        ),
        DeclareLaunchArgument(
            "gripper",
            default_value="none",
            description="Gripper name from za6_tools package; default none",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(
                        LaunchConfiguration("description_package")
                    ),
                    "config",
                    "initial_positions.yaml",
                ]
            ),
            description=("Path to initial positions YAML (for fake hardware)"),
        ),
        DeclareLaunchArgument(
            "robot_description_content",
            default_value=Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare(
                                LaunchConfiguration("description_package")
                            ),
                            "urdf",
                            LaunchConfiguration("description_file"),
                        ]
                    ),
                    " ",
                    "prefix:=",
                    LaunchConfiguration("prefix"),
                    " ",
                    "include_ros2_control:=",
                    LaunchConfiguration("include_ros2_control"),
                    " ",
                    "ros2_control_name:=",
                    LaunchConfiguration("ros2_control_name"),
                    " ",
                    "ros2_control_plugin:=",
                    LaunchConfiguration("ros2_control_plugin"),
                    " ",
                    "gripper:=",
                    LaunchConfiguration("gripper"),
                    " ",
                    "initial_positions_file:=",
                    LaunchConfiguration("initial_positions_file"),
                ]
            ),
            description=("URDF XML content, expanded from .xacro file"),
        ),
    ]

    return LaunchDescription(declared_arguments)
