# `za6_tools` package

Contains URDF file and meshes for grippers used by the ZA robot and
for Tormach machine tools.

## Referencing from another package URDF

From another package's URDF, attach a gripper to the robot's `tool0`
link:

```xml
<xacro:arg name="gripper" default="none"/>
<xacro:include filename="$(find za6_tools)/urdf/$(arg gripper)_macro.xacro"/>
<xacro:element xacro:name="xacro:${gripper}_macro"
               prefix="${prefix}"
               connected_to="${prefix}tool0"/>
```

## Referencing from another package `CMakeLists.txt`

From another package, grippers can be enumerated from
`CMakeLists.txt`, for example:

```cmake
find_package(za6_tools REQUIRED)
foreach(gripper ${za6_tools_grippers})
    set(gripper_urdf_macro ${za6_tools_urdf_dir}/${gripper}_macro.xacro)
    [...]
endforeach()
```

## Tool SRDF xacros

SRDFs for your robot can be generated with `xacro`.  The following
example can be added to the end of a robot SRDF.  The macro will
disable internal collisions for its own links.  Pass a list of robot
link names to disable collisions with the robot.

```xml
<xacro:arg name="gripper"/>
<xacro:include filename="$(find za6_tools)/config/$(arg gripper).srdf.xacro"/>

<!-- Disable collisions for links 5 & 6 -->
<xacro:element xacro:name="xacro:$(arg gripper)_macro"
               prefix="$(arg prefix)"
               robot_links="${['link_5', 'link_6']}"/>

<!-- Define end effector chain & link to main manipulator -->
<end_effector
    name="moveit_ee"
    parent_link="$(arg prefix)tool0"
    group="$(arg prefix)$(arg gripper)"/>
```
