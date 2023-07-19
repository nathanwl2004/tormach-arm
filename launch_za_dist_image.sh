#!/bin/bash -e
#
# *** Log into PathPilot Docker registry
#
# In order to pull the Docker image, first log in to the PathPilot
# Docker registry using your hub.pathpilot.com login credentials.
#
#    docker login docker.pathpilot.com
#    Username: <your email address>
#    Password: <your hub.pathpilot.com password>
#
# *** Launch the Tormach ZA6 ROS 2 container
#
# With the script in the current directory, start the container.  The
# first time will pull a new image, which may take several minutes.
# Add `sim` if no hardware is connected.
#
#   launch_za_dist_image.sh [sim]
#
# This starts a shell in the newly-launched Docker container.
# Additional container shells may be started in new terminals.
#
#   docker exec -itu $USER ros-dist bash
#
# *** Launch hardware, MoveIt and RViz
#
# Once the container is running, start the robot hardware, MoveIt
# configuration and RViz:
#
#   source /opt/ros/$ROS_DISTRO/setup.bash
#   ros2 launch za6_bringup bringup.launch
#
# Extra launch arguments:
#   hal_debug_level:=5    Enable verbose hardware debugging
#   sim_mode:=true        Run sim HAL hardware
#
# *** Enable drives
#
# The robot cannot move until motor power is supplied and drives are
# enabled.
#
# After powering on the robot control cabinet, supply motor power by
# releasing e-stop and pressing the reset button.  The reset button
# blue lamp will glow, indicating motor power is available.
#
# In software, after hardware launch, enable drives via the ROS
# service.  The main shell console will log detailed message about
# drive state changes, and the motor brakes will make audible clicks
# as they release.
#
#   source /opt/ros/$ROS_DISTRO/setup.bash  # If running a new terminal
#   ros2 service call /enable_drives std_srvs/srv/Trigger
#
# Disable drives with another ROS service.
#
#   ros2 service call /disable_drives std_srvs/srv/Trigger
#
# If drives fail to enable, look for clues in the main shell console
# logs.

if test "$1" != sim; then
    ETHERCAT=1
fi
NIC=enp2s0
MAC=$(ip link show $NIC | awk '/link\/ether/{print $2}')
DISPLAY=${DISPLAY:-:0}
case "$(cat /sys/devices/virtual/dmi/id/board_name)" in
*H310M*) RT_CPUS=5 ;;      # Beta controller
*YL-KBRL2*) RT_CPUS=3,7 ;; # Yanling KBRL2/N15 i5 controller
*)                         # Unknown; sim
    echo "WARNING Unknown motherboard; enabling sim mode" >&2
    ETHERCAT=
    ;;
esac
HUID=$(id -u)
HGID=$(id -g)
KVER=$(uname -r)
KSRC=(-v /usr/src/linux-headers-$KVER:/usr/src/linux-headers-$KVER)
DEB_KSRC=/usr/src/linux-headers-${KVER%%-rt-amd64}-common-rt
if test -d $DEB_KSRC; then
    KVER_MAJ_MIN=$(echo $KVER | awk -F . '{print $1 "." $2}')
    KSRC_KBUILD=/usr/src/linux-kbuild-$KVER_MAJ_MIN
    LIB_KBUILD=/usr/lib/linux-kbuild-$KVER_MAJ_MIN
    KSRC+=(
        -v $DEB_KSRC:$DEB_KSRC
        -v $KSRC_KBUILD:$KSRC_KBUILD
        -v $LIB_KBUILD:$LIB_KBUILD
    )
fi

set -x
exec docker run \
    --rm --privileged \
    --interactive --tty \
    --network host \
    --init \
    -e UID=$HUID -e GID=$HGID \
    -e QT_X11_NO_MITSHM=1 -e XDG_RUNTIME_DIR \
    -e HOME=$HOME \
    -e USER=$USER \
    -e TERM \
    -e ETHERCAT=$ETHERCAT \
    -e MAC=$MAC \
    -e DISPLAY=$DISPLAY \
    -e RT_CPUS=$RT_CPUS \
    -v /etc/timezone:/etc/timezone:ro \
    -v /etc/localtime:/etc/localtime:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/dri:/dev/dri \
    -v /boot:/boot \
    -v /var/lib/dkms/etherlabmaster:/var/lib/dkms/etherlabmaster \
    -v /lib/modules/$KVER:/lib/modules/$KVER \
    -v /var/run/docker.sock:/var/run/docker.sock \
    "${KSRC[@]}" \
    -v /dev:/dev \
    -v /ppdata:/ppdata \
    -v /run/user/$HUID:/run/user/$HUID \
    -v /ppdata:/ppdata \
    -v $HOME:$HOME \
    -v $PWD:$PWD \
    -w $PWD \
    -h ros-dist --name ros-dist \
    docker.pathpilot.com/ros2_public:humble-experimental-jammy-latest
