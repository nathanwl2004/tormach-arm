# `za6_description` package

URDF files and meshes for the ZA robot.

https://github.com/ros/xacro/
https://github.com/ros/urdf
https://wiki.ros.org/urdf

## Launch files in this package

- `robot_description.launch.py`:  Generate URDF
  - Included from other launch files, the URDF is available via the
    `robot_description_content` launch argument
  - Launch args:  (see file for complete list)
    - `gripper`:  Name of a gripper from the `za6_tools` package;
      default `none` (no gripper)
    - `prefix`:  Namespace prefix; default empty string

- `display_za6.launch.py`:  Display robot with joint slider controls UI
  - Launch command:  `ros2 launch za6_description display_za6.launch.py`
  - Launch args:  See `robot_description.launch.py` args, above

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
