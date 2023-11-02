# `za6_description` package

URDF files and meshes for the ZA robot.

https://github.com/ros/xacro/
https://github.com/ros/urdf
https://wiki.ros.org/urdf

## Launch files in this package

- `robot_description.launch.py`
  - Generates the URDF
  - Included from other launch files, the URDF is available via
    `LaunchConfiguration("robot_description_content")`
- `display_za6.launch.py`
  - Displays the robot with joint slider controls UI
  - Launch command:  `ros2 launch za6_description display_za6.launch.py`

## Xacro operations

- Convert xacro to URDF
  - `xacro -o /tmp/za6.urdf \
      $(ros2 pkg prefix za6_description)/share/za6_description/urdf/za6.xacro`
  - Optionally gripper, e.g. `gripper:=pivot_gripper`

- Verify URDF
  - `check_urdf /tmp/za6.urdf`

- Visualize URDF structure
  - `urdf_to_graphiz /tmp/za6.urdf`
  - Open `za6.pdf`
