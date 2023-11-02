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
