from launch import LaunchDescription

from launch_ros.actions import Node

from srdfdom.srdf import SRDF

from moveit_configs_utils import MoveItConfigsBuilder

# This launch file duplicates
# moveit_configs_utils.launches.generate_static_virtual_joint_tfs_launch()


def generate_launch_description():
    builder = MoveItConfigsBuilder("za6", package_name="za6_moveit_config")
    moveit_config = builder.to_moveit_configs()

    ld = LaunchDescription()

    name_counter = 0

    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            ld.add_action(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link,
                    ],
                )
            )
            name_counter += 1
    return ld
