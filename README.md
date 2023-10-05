# tormach_za6_studio_config

This contains the base and site config to operate a Tormach ZA6 using
MoveIt Studio

## Running the ZA6 on ROS 2

Run the ZA6, either real hardware or in sim mode, in a separate Docker
container using the instructions in this section.

NOTE:  Running ROS 2 will update the EtherCAT master, causing the ROS
1 controller to stop working.  See "Restore ROS 1 compatibility" below
for more info and a fix.

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

## Launch MoveIt Studio with ZA6 in simulation mode

- Start the ZA6 container as described above:

        ./launch_za_dist_image.sh sim

- Build a custom branch:

        rm -rf build/ log
        sudo -HE PYTHONPATH=$PYTHONPATH bash -c "
            colcon build --install-base /opt/ros/${ROS_DISTRO} \
                --merge-install --cmake-args
                -DCMAKE_BUILD_TYPE=Release \
                --event-handlers console_cohesion+ \
                --packages-skip za6_moveit_studio_config"

- Launch the ZA6 drivers:

        source /opt/ros/$ROS_DISTRO/setup.bash
        ros2 launch za6_bringup bringup.launch sim_mode:=true

In a new terminal:

- Remove previous MoveIt Studio configuration files

        rm -rf ~/.config/moveit_studio

- Build the ZA6 site config (from your moveit_studio folder):

       ./moveit_studio build

- Start MoveIt Studio (without launching drivers, they are launched in
  the ZA6 container):

       ./moveit_studio run -v --no-drivers
