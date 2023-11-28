#!/bin/bash -e
#
# See ../README.md for usage

if test "$1" = fix-ethercat; then
    PKG_VER=$(dpkg-query -W etherlabmaster-dkms | awk '{print $2}')
    sudo dkms install --force -m etherlabmaster -v $PKG_VER
    sudo systemctl restart ethercat
    exit
fi

if test "$1" != sim; then
    ETHERCAT=1
fi
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
NIC=enp2s0
MAC=$(ip link show $NIC 2>/dev/null | awk '/link\/ether/{print $2}')
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
DOCKER_IMAGE=docker.pathpilot.com/ros2_public:humble-experimental-jammy-latest

docker pull $DOCKER_IMAGE

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
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
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
    $DOCKER_IMAGE
