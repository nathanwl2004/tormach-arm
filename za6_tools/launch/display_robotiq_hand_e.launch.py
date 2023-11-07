import os
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_jspg = DeclareLaunchArgument(
        "use_joint_state_pub_gui",
        default_value="true",
        description="Start joint_state_publisher_gui node",
    )

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('za6_tools'),
            'urdf',
            'robotiq_hand_e.urdf.xacro',
        )
    )
    robot_description = dict(robot_description=robot_description_config.toxml())

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
          os.path.join(
              get_package_share_directory("za6_tools"),
              "rviz",
              "robotiq_hand_e.rviz",
          ),
      ],
      parameters=[robot_description]
      )

    return LaunchDescription(
        [
            use_jspg,
            rviz_node,
            robot_state_pub_node,
            joint_state_pub_gui_node,
        ]
    )
