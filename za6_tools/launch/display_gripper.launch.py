import os
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def create_robot_description(context):
    xacro_file = os.path.join(
        get_package_share_directory("za6_tools"),
        "urdf",
        context.launch_configurations["gripper"] + ".urdf.xacro",
    )
    assert os.path.exists(xacro_file), f"No such file, '{xacro_file}'"
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings=dict(
            prefix=context.launch_configurations["prefix"],
            connected_to=context.launch_configurations["connected_to"],
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

    use_jspg = DeclareLaunchArgument(
        "use_joint_state_pub_gui",
        default_value="true",
        description="Start joint_state_publisher_gui node",
    )


    robot_description_config = OpaqueFunction(function=create_robot_description)
    robot_description = LaunchConfiguration('robot_desc')

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
                  [LaunchConfiguration("gripper"), ".rviz"],
              ],
          ),
      ],
      parameters=[robot_description]
      )

    return LaunchDescription(
        [
            gripper,
            use_jspg,
            robot_description_config,
            rviz_node,
            robot_state_pub_node,
            joint_state_pub_gui_node,
        ]
    )
