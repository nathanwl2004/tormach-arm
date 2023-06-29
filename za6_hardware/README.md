# `za6_hardware`:  ZA robot HAL hardware configuration

This package contains the configuration and launch for the ZA robots
HAL hardware.  It configures and launches these things:

- The HAL realtime environment
- Hardware drivers, either EtherCAT (`lcec` HAL component) or sim
- `hw_device_mgr`, ROS2 node for controlling EtherCAT drive state in HAL
  - See the [upstream project][hw_device_mgr] and the [development
    branch used here][hw_device_mgr_dev]
- `hal_hw_interface`, `ros2_control` controller manager in HAL
  - From `hal_ros_control`; see the [upstream
    project][hal_ros_control] and the [development branch used
    here][hal_ros_control_dev]
- `hal_io`, ROS2 node for connecting IO & other HAL pins to ROS topics
  - Also from `hal_ros_control`

[hw_device_mgr]:  https://github.com/tormach/hw_device_mgr
[hw_device_mgr_dev]:  https://github.com/zultron/hw_device_mgr/tree/zultron/2023-05-02-overlapping-pdos
[hal_ros_control]:  https://github.com/tormach/hal_ros_control
[hal_ros_control_dev]:  https://github.com/zultron/hal_ros_control/tree/zultron/2023-04-14-humble-devel


## Bring up hardware

To launch the hardware (no UI):

    ros2 launch za6_hardware hal_hardware.launch.py

Add optional args:  (for complete list, see `launch/hal_hardware.launch.py`)

    sim_mode:=true  # Run sim HAL hardware
    hal_debug_output:=true hal_debug_level:=5  # HAL verbose logging to console


## Command drive state

The `hw_device_mgr` controls drive state.  A ROS node, `/drive_state`,
presents services to enable or disable drives and to query state.

Check current drive state:  enabled or faulted.

    ros2 topic echo --once /drives_enabled std_msgs/msg/Bool
    ros2 topic echo --once /drives_faulted std_msgs/msg/Bool

Zero command - feedback error.

    ros2 service call /zero_error std_srvs/srv/Trigger

Enable or disable drives.

    # Enable drives (also zeros error)
    ros2 service call /enable_drives std_srvs/srv/Trigger
    # Disable drives
    ros2 service call /disable_drives std_srvs/srv/Trigger

The `hw_device_mgr` will also log detailed diagnostic messages to the
console.

## Dump drive params

A complete list of a particular drive's params can be dumped with the
following command.  The `<drive_pos>` argument is 0-5, corresponding
to joints 1-6, respectively.

    ros2 run za6_hardware dump_params <drive_pos>
