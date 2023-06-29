from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

# This launch file duplicates
# moveit_configs_utils.launches.generate_rsp_launch()


def generate_launch_description():
    """Launch file for robot state publisher (rsp)"""
    builder = MoveItConfigsBuilder("za6", package_name="za6_moveit_config")
    moveit_config = builder.to_moveit_configs()

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument("publish_frequency", default_value="15.0")
    )

    # Given the published joint states, publish tf for the robot links and the
    # robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )
    ld.add_action(rsp_node)

    return ld
