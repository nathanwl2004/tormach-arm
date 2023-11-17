#!/bin/bash -e

log() {
    echo "DOCKER_DEV:  $*" >&2
}

usage() {
    if test -z "$*"; then
        RC=0
    else
        echo "Error:  $*" >&2
        RC=1
    fi
    cat >&2 <<EOF
Usage: $0 [-b|-B|-H|-p|-I] [args]
    Run the container:  (default)
      -d:  Run the distribution image (default:  development image)
      -l:  Run PathPilot launcher
      -lv:  Run Virtual PathPilot launcher
      -N:  Don't run the overlay image (conflicts with -d)
      -t IMAGE:  Specify the complete image tag
      -i IMAGE_VERSION:  Specify the image version
      -G:  Run the last-known-good development image (conflicts with -i)
      -n NAME:  Set container name and hostname to NAME
      -L LINK:  Link to container named LINK
      -S:  Request usage of Freedesktop Secret Service for storing secrets
      CMD [ARG ...]:  Run command with args (default:  interactive shell)
    Build the image:  -b
      -d:  Build the distribution image (default:  development image)
      -N:  Don't build the overlay image (conflicts with -d)
      -t IMAGE:  Specify the complete image name
      -T TARGET:  Build TARGET from Dockerfile (default:  ros_devel or ros_dist)
      -m ROBOT_MODEL:  Selects \${ROBOT_MODEL}_moveit_config robot config pkg
      -P ROBOT_PACKAGE:  Selects \${ROBOT_PACKAGE} robot config pkg
      -g:  Ignore dirty workdir *for testing only*
      -- ARGS ...:  Add ARGS to the 'docker build' command
    Bump the image version:  -B or -H
      -B:  Bump both image version and image hash
      -H:  Bump image hash only
    Push the built image:  -p
      -d:  Push the distribution image (default:  development image)
      -N:  Don't push the overlay image (conflicts with -d)
    Pull latest image from Docker registry: -D
      -d:  Pull the distribution image (default:  development image)
    Export the built artifact from DIST image: -E PATH
      -E PATH:  Path in host filesystem to copy artifacts (must already exist)
    Tag an image 'latest':  -A
      -d:  Tag the distribution image (default:  development image)
      -t IMAGE:  Specify the complete image tag
      -i IMAGE_VERSION:  Specify the image version
    Configuration information:  -I
EOF
    exit $RC
}

while getopts :dt:Si:N-bT:m:P:gpBHlvn:L:GchDE:AI ARG; do
    case $ARG in
    # Global options
    d)
        IMAGE_TYPE=dist
        USE_OVERLAY=false
        ;;
    t)
        IMAGE_BASE=$OPTARG
        USE_OVERLAY=false
        test -z "$IMAGE_VERSION" || usage "Conflicting options '-i' and '-t'"
        ;;
    i)
        IMAGE_VERSION=$OPTARG
        test -z "$IMAGE_BASE" || usage "Conflicting options '-i' and '-t'"
        ;;
    N) USE_OVERLAY=false ;;
    -) break ;; # Following args passed to docker command
    # Build mode
    b) BUILD=true ;;
    T) # For splitting up build:  tag is stage name; drop version
        TARGET=$OPTARG
        IMAGE_TAG=$TARGET
        USE_OVERLAY=false
        ;;
    m)
        ROBOT_MODEL=$OPTARG
        ;;
    P)
        ROBOT_PACKAGE=$OPTARG
        ;;
    g)
        IGNORE_DIRTY_WORKDIR=1
        ;;
    # Push mode
    p) PUSH=true ;;
    # Version bump
    B)
        BUMP_IMAGE_VERSION=true
        WANT_ENV='bare-metal docker-run'
        ;;
    H)
        BUMP_IMAGE_HASH=true
        WANT_ENV='bare-metal docker-run'
        ;;
    # Run options
    l) LAUNCHER=true ;;
    v) VIRTUAL_PATHPILOT=true ;;
    n) NAME=$OPTARG ;;
    L) LINK_CONTAINER=$OPTARG ;;
    G)
        LAST_KNOWN_GOOD=true
        if test -f $HOME/.pathpilot_ros2.lng; then
            IMAGE_VERSION=$(sed $HOME/.pathpilot_ros2.lng -e 's/\./+/')
        fi
        ;;
    # Misc
    I)
        PRINT_INFO=true
        WANT_ENV=any
        ;;
    E)
        ARTIFACT_EXPORT=true
        ARTIFACT_EXPORT_PATH="${OPTARG}"
        # So far the DEVEL images have nothing to export
        IMAGE_TYPE=dist
        ;;
    A) TAG_IMAGE=true ;;
    c) CHECK_ENV=true ;;
    D) PULL=true ;;
    S) SECRET_SERVICE=True ;;
    h) usage ;;
    :) usage "Option -$OPTARG requires an argument" ;;
    *) usage "Illegal option -$OPTARG" ;;
    esac
done
shift $(($OPTIND - 1))

# Set params
# - PathPilot launcher?  (-l)
LAUNCHER=${LAUNCHER:-false}
# - Virtual PathPilot? (-v)
VIRTUAL_PATHPILOT=${VIRTUAL_PATHPILOT:-false}
# - Build image? (-b)
BUILD=${BUILD:-false}
# - Image type:  'dist' or 'devel'
IMAGE_TYPE=${IMAGE_TYPE:-devel}
# - Bump image version? (-B)
BUMP_IMAGE_VERSION=${BUMP_IMAGE_VERSION:-false}
# - Bump image hash? (-H)
BUMP_IMAGE_HASH=${BUMP_IMAGE_HASH:-false}
# - Push image? (-p)
PUSH=${PUSH:-false}
# - Build target (-T)
DEFAULT_TARGET=ros_${IMAGE_TYPE}
TARGET=${TARGET:-${DEFAULT_TARGET}}
# - Robot model (-m)
DEFAULT_ROBOT_MODEL=za
ROBOT_MODEL=${ROBOT_MODEL:-${DEFAULT_ROBOT_MODEL}}
# - Robot package (-P)
DEFAULT_ROBOT_PACKAGE=za6_robot
ROBOT_PACKAGE=${ROBOT_PACKAGE:-${DEFAULT_ROBOT_PACKAGE}}
# - Container name (-n)
NAME=${NAME:-ros2-${IMAGE_TYPE}}
# - Container name to link (-L NAME):  for sim
LINK_CONTAINER=${LINK_CONTAINER:+--link=${LINK_CONTAINER}}
# - Print information (-I)
PRINT_INFO=${PRINT_INFO:-false}
# - Tag image (-A)
TAG_IMAGE=${TAG_IMAGE:-false}
# - Check if running bare metal? (-c)
CHECK_ENV=${CHECK_ENV:-false}
# - Run last-known-good container
LAST_KNOWN_GOOD=${LAST_KNOWN_GOOD:-false}
# - Pull the latest image from Docker registry
PULL=${PULL:-false}

# Read common settings
WANT_ENV="${WANT_ENV:-bare-metal}"
if ${CHECK_ENV}; then
    WANT_ENV_CHECK="${WANT_ENV}"
    WANT_ENV=
fi
source "$(dirname $0)/env.sh"

###########################
# Check wanted environment
if ${CHECK_ENV}; then
    if test_environment ${WANT_ENV_CHECK}; then
        exit 0 # outside docker
    else
        exit 1 # in docker
    fi
fi

###########################
# Docker pull latest image
if $PULL; then
    set -x
    log "Pulling image ${IMAGE}"
    exec ${DO} docker pull ${IMAGE}
    exit -1 # Should never get there
fi

###########################
# Export built artifacts
if [[ "$ARTIFACT_EXPORT" == "true" ]]; then
    log "Requested EXPORT of artifacts"
    if [[ ! -d ${ARTIFACT_EXPORT_PATH} ]]; then
        log "Path ${ARTIFACT_EXPORT_PATH} does not exist or is not a directory"
        exit 1
    fi

    DOCKER_CREATE_ID=$(docker create ${IMAGE})
    docker cp ${DOCKER_CREATE_ID}:/opt/pathpilot/robot/packages/deb ${ARTIFACT_EXPORT_PATH}
    log "Export successful, discarding container"
    docker container rm ${DOCKER_CREATE_ID}
    log "Exiting"
    exit 0
fi

###########################
# Bump image version
if ${BUMP_IMAGE_VERSION} || ${BUMP_IMAGE_HASH}; then
    log "Bumping image version"
    save_image_version ${BUMP_IMAGE_HASH}
    $BUILD || exit $(test -z ${IMAGE_VERSION_UPDATED})
fi

###########################
# Print information
if ${PRINT_INFO}; then
    echo "image_version ${IMAGE_VERSION}"
    echo "image ${IMAGE}"
    echo "using_overlay ${USE_OVERLAY}"
    if test -z "$SKIP_HW_CHECK"; then
        check_hardware
        echo "hardware_supported ${HW_SUPPORTED}"
        echo "hardware ${HW_BVENDOR}:${HW_PNAME}"
        echo "rt_cpus ${RT_CPUS}"
    fi
    echo "robot_model ${ROBOT_MODEL}"
    exit 0
fi

###########################
# Tag image
if $TAG_IMAGE; then
    log "Tagging image ${IMAGE} as ${IMAGE_TAG_LATEST}"
    docker tag ${IMAGE} ${IMAGE_TAG_LATEST}
    exit 0
fi

###########################
# Docker build base or overlay image
if $BUILD; then
    # Don't build top-level image unless image version looks good
    test $TARGET != $DEFAULT_TARGET || check_image_version -f

    cur_dir=$(realpath $(dirname $0))
    RELEASE_VERSION=$(docker run --rm -v "$cur_dir:/tmp/version" \
        -w "/tmp/version" python:3.9.1 python3 \
        ppr_release.py get v -l)
    GIT_SHORT_SHA=$(in_repo_dir git rev-parse --short HEAD)

    if ! ${USE_OVERLAY}; then
        # Build base image when -N is supplied
        # - Use a local mirror if specified
        test -z "${DEBIAN_MIRROR}" || DOCKER_DEV_BUILD_OPTS+=(
            --build-arg DEBIAN_MIRROR=${DEBIAN_MIRROR})
        test -z "${DEBIAN_SECURITY_MIRROR}" || DOCKER_DEV_BUILD_OPTS+=(
            --build-arg DEBIAN_SECURITY_MIRROR=${DEBIAN_SECURITY_MIRROR})
        test -z "${HTTP_PROXY}" || DOCKER_DEV_BUILD_OPTS+=(
            --build-arg HTTP_PROXY=${HTTP_PROXY})
        # - Add git rev label and ENV to top-level images
        test "${TARGET}" != "${DEFAULT_TARGET}" || DOCKER_DEV_BUILD_OPTS+=(
            --build-arg GIT_REV=${GIT_SHORT_SHA})
        # - Add robot model and package to label and environment
        DOCKER_DEV_BUILD_OPTS+=(
            --label com.tormach.pathpilot.robot.model=${ROBOT_MODEL}
            --label com.tormach.pathpilot.robot.package=${ROBOT_PACKAGE}
            --build-arg ROBOT_MODEL=${ROBOT_MODEL}
            --build-arg ROBOT_PACKAGE=${ROBOT_PACKAGE}
        )
        # - Use BuildKit backend
        export DOCKER_BUILDKIT=1
        # - Docker build
        set -x
        exec ${DO} docker build -t "${IMAGE_BASE}" \
            --target=${TARGET} \
            --build-arg ROS_DISTRO=$ROS_DISTRO \
            --build-arg OS_VENDOR=$OS_VENDOR \
            --build-arg DEBIAN_SUITE=$DEBIAN_SUITE \
            --build-arg BASE_OS_DOCKER_IMAGE=$BASE_OS_DOCKER_IMAGE \
            --build-arg IMAGE_VERSION=$IMAGE_VERSION \
            --build-arg RELEASE_VERSION=${RELEASE_VERSION} \
            --progress=plain \
            --network host \
            "${DOCKER_DEV_BUILD_OPTS[@]}" \
            ${BUILD_ARGS} \
            "$@" -f ${DOCKER_SCRIPTS_DIR}/Dockerfile "${REPO_DIR}"
    else
        # Build the overlay image when -N isn't supplied
        # - OVERLAY_DIR may be set in the rc file or the environment
        test -z "${OVERLAY_DIR}" && usage "Please specify overlay directory"
        test -d "${OVERLAY_DIR}" ||
            usage "Overlay directory ${OVERLAY_DIR} does not exist"
        # - Docker build
        set -x
        exec ${DO} docker build -t "${IMAGE_OVERLAY}" \
            --build-arg IMAGE_BASE="${IMAGE_BASE}" \
            --build-arg RELEASE_VERSION=${RELEASE_VERSION} \
            ${OVERLAY_DOCKER_PATH:+--file=${OVERLAY_DOCKER_PATH}} \
            "${DOCKER_DEV_OVERLAY_BUILD_OPTS[@]}" \
            ${OVERLAY_BUILD_ARGS} \
            "$@" \
            "${OVERLAY_DIR}"
    fi
fi

###########################
# Docker push
if $PUSH; then
    set -x
    exec ${DO} docker push ${IMAGE}
fi

###########################
# Docker run

# Query for user data to pass as the PathPilot Robot container user
C_UID=$(id -u)
C_GID=$(id -g)
C_UN=$(id -un)
C_UH=$(getent passwd $C_UN | cut -d: -f6)

# Additional arguments to pass to 'docker run' command
declare -a DOCKER_RUN_OPTS
DOCKER_RUN_OPTS=(${DOCKER_ARGS})

# Args for interactive `docker run` or `docker exec`
declare -a DOCKER_INTERACTIVE

# Temporary run directory for this instance
RUN_TEMP_DIRECTORY="/tmp/.pathpilotrobot-$(date +%s)"
DOCKER_RUN_TEMP_DIRECTORY="/tmp/.pathpilotrobotrun"

mkdir -p ${RUN_TEMP_DIRECTORY}
DOCKER_RUN_OPTS+=(-v ${RUN_TEMP_DIRECTORY}:${DOCKER_RUN_TEMP_DIRECTORY})

# In devel environment, use entrypoint.sh from git repo
DEVEL_ENTRYPOINT="${DOCKER_SCRIPTS_DIR}/ros_d_common/entrypoint.sh"
if $IN_WS_DIR && [[ -f "$DEVEL_ENTRYPOINT" ]]; then
    log "Using entrypoint from workspace:  $DEVEL_ENTRYPOINT"
    DOCKER_RUN_OPTS+=(--entrypoint "$DEVEL_ENTRYPOINT")
fi

# Execute a ROS setup script from ROS devel configuration if exists
# (only for DEVEL type of images, DIST ones should be encapsulated)
if [[ "$IMAGE_TYPE" == "devel" ]]; then
    DEVEL_SETUP_SCRIPT="${WS_DIR}/install/setup.bash"
    if $IN_WS_DIR && [[ -f "$DEVEL_SETUP_SCRIPT" ]]; then
        log "Sourcing workspace setup script:  $DEVEL_SETUP_SCRIPT"
        DOCKER_RUN_OPTS+=(-e ROS_SETUP="$DEVEL_SETUP_SCRIPT")
    fi
fi

# Warn if image looks stale (or else suppress the warning)
if test -z "$SKIP_IMAGE_CHECK"; then
    check_image_version
else
    DOCKER_RUN_OPTS+=(-e SKIP_IMAGE_CHECK=1)
fi

# Check hardware and set environment
if test -z "$SKIP_HW_CHECK"; then
    check_hardware
else
    DOCKER_RUN_OPTS+=(-e SKIP_HW_CHECK=1)
fi

# PathPilot launcher
if ${LAUNCHER}; then
    log "Running PathPilot launcher"
    CONTAINER_NAME=${NAME}-launch
    DOCKER_RUN_OPTS+=(-e LAUNCHER=1)
else
    CONTAINER_NAME=${NAME}
fi

# PathPilot on real display
DOCKER_RUN_OPTS+=(
    -e DISPLAY
    -e RT_CPUS=$RT_CPUS
    -v /tmp/.X11-unix:/tmp/.X11-unix
    -v /dev/dri:/dev/dri
    --network host
)

if ! ${LAUNCHER}; then
    # PathPilot running from terminal
    DOCKER_RUN_OPTS+=(
        --init # Run init to clean up zombies
    )

fi

if [[ -v SECRET_SERVICE && "$SECRET_SERVICE" == "True" ]]; then
    DOCKER_RUN_OPTS+=(-e PATHPILOT_USE_SECRET_SERVICE="True")
fi

# Check for existing containers
EXISTING="$(docker ps -aq --filter=name=^/${CONTAINER_NAME}$)"
RUNNING=false
if test -n "${EXISTING}"; then
    # Container exists; is it running?
    RUNNING=$(docker inspect $CONTAINER_NAME |
        awk -F '[ ,]+' '/"Running":/ { print $3 }')
    if test "${RUNNING}" = "false"; then
        log "Container '${CONTAINER_NAME}' already exists; stopping"
        docker rm ${CONTAINER_NAME}
    elif test "${RUNNING}" = "true"; then
        log "Container '${CONTAINER_NAME}' already running"
    else
        # Something went wrong
        log "Error:  unable to determine status of " \
            "existing container '${EXISTING}'"
        exit 1
    fi
elif test -z "$SKIP_HW_CHECK"; then
    # Container doesn't exist yet; detect hardware

    # - Video driver
    case "${OGL_VENDOR}::${OGL_RENDERER}" in
    Intel::*) # Intel graphics; not sure why different from below
        log "Detected Intel graphics card"
        # No special config
        ;;
    "Intel Open Source Technology Center"::*) # Brix; John's ThinkPad X201t
        log "Detected Intel graphics card"
        # No special config
        ;;
    "NVIDIA Corporation"::*) # Alexander's NVidia w/proprietary drivers
        log "Detected NVIDIA graphics card"
        DOCKER_RUN_OPTS+=(
            --runtime=nvidia
            -e NVIDIA_VISIBLE_DEVICES=all
            -e NVIDIA_DRIVER_CAPABILITIES=graphics
        )
        ;;
    "VMware, Inc."::*) # SSH forwarded X connection
        # (Probably will never see this)
        log "Detected virtual graphics hardware"
        log "WARNING:  The robot_ui will not run with this hardware!"
        ;;
    "nouveau"::*) # Bas's NVidia w/FOSS drivers
        log "Detected Nouveau driver"
        # No special config
        ;;
    "X.Org"::*POLARIS*) # Rob's AMD RX 580 GPU
        log "Detected Polaris driver"
        DOCKER_RUN_OPTS+=(-v /dev/kfd:/dev/kfd)
        ;;
    *)
        log "Unable to detect graphics hardware"
        log "WARNING:  The robot_ui may not run with this hardware!"
        log "Please add your hardware to the '$0' script"
        ;;
    esac

    # - Hardware mode
    eth_ip() {
        ip addr show $1 2>/dev/null |
            grep -o "inet [0-9]*\.[0-9]*\.[0-9]*\.[0-9]*" |
            grep -o "[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*" ||
            true
    }
    HM2IP=192.168.1.1
    KHDR_DIR=/usr/src/linux-headers-$(uname -r)
    if test -e /dev/EtherCAT0 -a -d $KHDR_DIR; then
        # HACK:  If EtherCAT master running on host, assume ethercat.
        # Otherwise, depend on $ETHERCAT being set elsewhere.
        export ETHERCAT=1
    fi
    if test -n "$ETHERCAT"; then
        # Add EtherCAT DKMS support
        log "Detected EtherCAT hardware"
        DOCKER_RUN_OPTS+=(
            -v /boot:/boot
            -v /var/lib/dkms/etherlabmaster:/var/lib/dkms/etherlabmaster
            -v /lib/modules/$(uname -r):/lib/modules/$(uname -r)
            -v $KHDR_DIR:$KHDR_DIR
            -e MAC=${MAC^^}
        )
    elif test "$(eth_ip eth0)" = $HM2IP -o "$(eth_ip enp1s0)" = $HM2IP -o "$(eth_ip enp2s0)" = $HM2IP; then
        log "Detected Hostmot2 hardware"
    else
        log "Failed to detect EtherCAT or Hostmot2 hardware; default to sim mode"
    fi

    # USB devices
    DOCKER_RUN_OPTS+=(-v /dev:/dev)
fi

if test -n "$ENV_CI"; then
    # Pass $ENV_CI for tests
    DOCKER_RUN_OPTS+=(-e ENV_CI)
fi

if ((C_UID != 0 & C_GID != 0)); then
    # CI 'dist' image tests run as root, so only mount $HOME for
    # non-root
    DOCKER_RUN_OPTS+=(-v $HOME:$HOME)
    if [[ "$HOME" != "$C_UH" ]]; then
        DOCKER_RUN_OPTS+=(-v $C_UH:$C_UH)
    fi
    # usb bind-propagation to mount usb media mount directory
    USB_MEDIA_PATH=/media/${USER}
    if [ -d "$USB_MEDIA_PATH" ]; then
        DOCKER_RUN_OPTS+=(--mount type=bind,source=${USB_MEDIA_PATH},target=${USB_MEDIA_PATH},bind-propagation=shared)
    fi
fi
if test -t 1; then
    # attached to tty; set --interactive
    DOCKER_RUN_OPTS+=(--interactive)
    DOCKER_INTERACTIVE+=(--interactive)
fi
CMD_SUPPLIED=false # Was CMD supplied as script args?
if test -n "$*"; then
    # CMD was supplied as script args; separate stdout + stderr
    DOCKER_RUN_OPTS+=(-a stdin -a stdout -a stderr -e CMD_SUPPLIED=1)
elif test -t 1; then
    # CMD was not supplied; tty attached; allocate a tty for container
    DOCKER_RUN_OPTS+=(--tty)
    DOCKER_INTERACTIVE+=(--tty)
fi

if test -d /ppdata; then
    DOCKER_RUN_OPTS+=(-v /ppdata:/ppdata)
fi

if test -n "$XDG_RUNTIME_DIR"; then
    DOCKER_RUN_OPTS+=(-v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR)
fi

DOCKER_RUN_OPTS+=(-e RC_FILE=${DOCKER_RUN_TEMP_DIRECTORY}/rc.sh)

if ${VIRTUAL_PATHPILOT}; then
    # Virtual PathPilot over VNC
    set -x
    exec ${DO} docker run --rm \
        --interactive \
        --tty \
        --env VIRTUAL_PATHPILOT=1 \
        --publish=5900:5900 \
        --hostname virtual-pathpilot \
        --name virtual-pathpilot \
        ${IMAGE}

else
    # Give the user a shall
    if ! ${RUNNING}; then
        # No existing container; start new one

        set -x
        exec ${DO} docker run --rm \
            --privileged \
            -e UID=${C_UID} \
            -e GID=${C_GID} \
            -e QT_X11_NO_MITSHM=1 \
            -e XDG_RUNTIME_DIR \
            -e HOME=${C_UH} \
            -e USER=${C_UN} \
            -e TERM \
            -e ETHERCAT=$ETHERCAT \
            -v $PWD:$PWD \
            -e DBUS_SESSION_BUS_ADDRESS \
            -v /var/run/systemd/journal/socket:/var/run/systemd/journal/socket \
            -v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket \
            -v /var/run/docker.sock:/var/run/docker.sock \
            -v /etc/timezone:/etc/timezone:ro \
            -v /etc/localtime:/etc/localtime:ro \
            -w $PWD \
            -h ${CONTAINER_NAME} --name ${CONTAINER_NAME} \
            "${DOCKER_RUN_OPTS[@]}" \
            ${LINK_CONTAINER} \
            ${IMAGE} "$@"
    else
        # Container already started:  Exec a new shell in the existing container
        if test -z "$*"; then
            set -x
            exec ${DO} docker exec "${DOCKER_INTERACTIVE[@]}" -tu ${USER} ${CONTAINER_NAME} bash -i
        else
            set -x
            exec ${DO} docker exec "${DOCKER_INTERACTIVE[@]}" -tu ${USER} ${CONTAINER_NAME} "$@"
        fi
    fi
fi
