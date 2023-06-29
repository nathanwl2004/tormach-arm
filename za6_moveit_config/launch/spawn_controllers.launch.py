from launch import LaunchDescription

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

# This launch file duplicates
# moveit_configs_utils.launches.generate_spawn_controllers_launch()


def generate_launch_description():
    builder = MoveItConfigsBuilder("za6", package_name="za6_moveit_config")
    moveit_config = builder.to_moveit_configs()

    controller_mgr_config = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    )
    controller_names = controller_mgr_config.get("controller_names", [])

    ld = LaunchDescription()

    for controller in controller_names + ["joint_state_broadcaster"]:
        params = controller_mgr_config.get(controller, dict())
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                parameters=[params],
                output="screen",
            )
        )
    return ld
