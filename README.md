# `tormach_za_drivers`

This repository contains the Tormach ZA robot ROS 2 drivers,
including:

- `za6_description`:  URDF and mesh files for the ZA6
- `za6_tools`:  URDF and meshfiles for Tormach robot grippers
- `za6_hardware`:  HAL hardware drivers for the ZA6
- `za6_moveit_config`:  MoveIt configuration
- `za6_bringup`:  Launch files to bring up the robot

It also includes two packages for running MoveIt Studio:

- `moveit_studio_za6_base_config`:  The base Studio configuration
- `moveit_studio_za6_tending_config`:  The machine tending
  configuration with a Tormach PCNC 1100 milling machine

## Running the ZA6 on ROS 2

Run the ZA6, either real hardware or in sim mode, in a Docker
container using the instructions in this section.  Real hardware will
only run on a Tormach robot controller with `RT_PREEMPT` kernel and OS
tuning needed for real-time control.

NOTE:  Running ROS 2 will update the EtherCAT master, causing the ROS
1 controller to stop working.  See "Restore ROS 1 compatibility" below
for more info and a fix.  This will be fixed in future releases.

### Log into PathPilot Docker registry

First, create an account at https://hub.pathpilot.com/ and ask John
Morris to grant access to the `docker.pathpilot.com/ros2_public`
registry namespace.

To pull the Docker image, first log in to the PathPilot Docker
registry using your hub.pathpilot.com login credentials.

    docker login docker.pathpilot.com
    Username: <your email address>
    Password: <your hub.pathpilot.com password>

### Launch the Tormach ZA6 ROS 2 container

Start the container with the `launch_za_dist_image.sh` script.  The
first time will pull a new image, which may take several minutes.  Add
`sim` if no hardware is connected.

    ./launch_za_dist_image.sh [sim]

This starts a shell in the newly-launched Docker container.
Additional container shells may be started in new terminals.

    docker exec -itu $USER ros-dist bash

### Launch hardware, MoveIt and RViz

Once the container is running, start the robot hardware, MoveIt
configuration and RViz:

    source /opt/ros/$ROS_DISTRO/setup.bash
    ros2 launch za6_bringup bringup.launch

Extra launch arguments:

    hal_debug_level:=5    Enable verbose hardware debugging
    sim_mode:=true        Run sim HAL hardware

### Enable drives

The robot cannot move until motor power is supplied and drives are
enabled.

After powering on the robot control cabinet, supply motor power by
releasing e-stop and pressing the reset button.  The reset button
blue lamp will glow, indicating motor power is available.

In software, after hardware launch, enable drives via the ROS
service.  The main shell console will log detailed message about
drive state changes, and the motor brakes will make audible clicks
as they release.

    source /opt/ros/$ROS_DISTRO/setup.bash  # If running a new terminal
    ros2 service call /enable_drives std_srvs/srv/Trigger

Disable drives with another ROS service.

    ros2 service call /disable_drives std_srvs/srv/Trigger

If drives fail to enable, look for clues in the main shell console
logs.

## Restore ROS 1 compatibility

The ROS 1 containers depend on an older EtherCAT master running on
the host.  The ROS 2 container updates the master when the Docker
container starts.  To revert the EtherCAT master to the older
version for running ROS 1, run this script with the `fix-ethercat`
argument.

  launch_za_dist_image.sh fix-ethercat

## Launch MoveIt Studio with ZA6 support

To run MoveIt Studio, see `moveit_studio_za6_base_config/README.md`.
