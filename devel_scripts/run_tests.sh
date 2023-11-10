#!/bin/bash -e
#
# Run unit tests in Docker

usage() {
    if test -z "$*"; then
        RC=0
    else
        echo "Error:  $*" >&2
        RC=1
    fi
    cat >&2 <<EOF
Usage:  $0 [args...]
    -C:  Run with CI settings (like -gUfxbdt)
    -g:  Clean and reset git tree WARNING:  DESTROYS UNCOMMITTED CHANGES
    -U:  Update pre-commit formatters
    -f:  Run formatters
    -x:  Run dummy Xorg server (for headless operation)
    -b:  Build workspace (devel image only)
    -d:  Update rosdep
    -t:  Run package tests
    -D:  Build rosdep docs
    -y:  'Yes' mode: don't prompt
    -h:  Show this help message
EOF
    exit $RC
}
if test -z "${ENV_CI}" -a -z "$*"; then
    usage
fi

BASE_SETUP_SCRIPT=/opt/ros/${ROS_DISTRO}/setup.bash
if test $IMAGE_TYPE = dist; then
    SETUP_SCRIPT=$BASE_SETUP_SCRIPT
    DEVEL_IMAGE=false
else
    SETUP_SCRIPT=install/setup.bash
    DEVEL_IMAGE=true
fi

set_all() {
    # Do the full shebang (except docs)
    SHOW_DEBUGGING=true
    CLEAN_GIT=true
    UPDATE_PRE_COMMIT=$DEVEL_IMAGE # no dev tools in dist image
    RUN_FORMATTERS=$DEVEL_IMAGE
    RUN_XORG=true
    BUILD_WORKSPACE=$DEVEL_IMAGE # workspace is in-built in dist image
    UPDATE_ROSDEP=true
    RUN_TESTS=true
    BUILD_DOCS=true
}

while getopts :CgUfxbdltDyh ARG; do
    case $ARG in
    C) set_all ;;
    g) CLEAN_GIT=true ;;
    U) UPDATE_PRE_COMMIT=true ;;
    f) RUN_FORMATTERS=true ;;
    x) RUN_XORG=true ;;
    b) BUILD_WORKSPACE=true ;;
    d) UPDATE_ROSDEP=true ;;
    t) RUN_TESTS=true ;;
    D) BUILD_DOCS=true ;;
    y) YES_MODE=true ;;
    h) usage ;;
    *) usage "Unknown option '-$ARG'" ;;
    esac
done
shift $(($OPTIND - 1))

if test -n "${ENV_CI}"; then
    # Do the full shebang in CI mode, no questions asked
    echo "Running in CI mode"
    set_all
    YES_MODE=true
fi

if ${SHOW_DEBUGGING:-false}; then
    # Show installed software versions in CI for debugging
    echo "Installed Debian packages:"
    dpkg-query -W
    echo
    echo "Installed pip3 packages:"
    pip3 list --format=columns
fi

# Show what we're doing
set -x

if ${CLEAN_GIT:-false}; then
    # Clean up ignored files; uncommitted files aren't touched to make
    # this safer in a dev environment
    #
    # First, fix this error:
    # git clean -Xdf
    # fatal: detected dubious ownership in repository at '/opt/buildagent/work/185fff4ceaae512b'
    git config --global --add safe.directory $PWD
    git clean -Xdf
fi

if ${UPDATE_PRE_COMMIT:-false}; then
    pre-commit autoupdate
fi

if ${RUN_FORMATTERS:-false}; then
    # Run formatters
    pre-commit run --all-files || FAIL=1
    if test -n "${FAIL}"; then
        test -z "${ENV_CI}" || (git diff | head -500) # Show diff in CI
        exit 1
    fi
fi

if ${RUN_XORG:-false}; then
    # Set up X server for robot_ui tests
    sudo Xorg -noreset +extension GLX +extension RANDR +extension RENDER \
        -logfile /var/log/xorg.log -config /etc/pathpilot/xorg.conf :1 &
    export DISPLAY=:1
fi

if ${BUILD_WORKSPACE:-false}; then
    # Build the workspace
    source $BASE_SETUP_SCRIPT
    VERBOSE=1 \
        colcon build \
        --event-handlers console_cohesion+ \
        --cmake-args -Werror=dev
fi

if ${UPDATE_ROSDEP:-false}; then
    # Update rosdep sources
    rosdep update
fi

if ${RUN_TESTS:-false}; then
    # Build and run the tests
    source $BASE_SETUP_SCRIPT
    colcon build \
        --event-handlers console_cohesion+ \
        --cmake-args -Werror=dev
    source $SETUP_SCRIPT
    # Ughly hack
    PYTHONPATH+=":/opt/ros/humble/local/lib/python3.10/dist-packages"
    VERBOSE=1 \
        colcon test \
        --event-handlers console_cohesion+
    source $SETUP_SCRIPT
    colcon test-result --verbose
fi

if ${BUILD_DOCS:-false}; then
    # Build rosdoc2 docs
    source $SETUP_SCRIPT
    had_error=false
    PKG_PATHS=($(for f in $(find src/ -name package.xml); do dirname $f; done))
    for PPATH in "${PKG_PATHS[@]}"; do
        echo Package path: $PPATH
        test ! -e $PPATH/COLCON_IGNORE || continue
        if test -f ${PPATH}/rosdoc.yaml; then
            { err=$(rosdoc2 build --package-path $PPATH 2>&1 >&3 3>&-); } 3>&1
            echo >&2 "${err}"
            if echo ${err} | grep "ERROR"; then
                had_error=true
            fi
        fi
    done
    if ${had_error}; then
        echo >&2 "Error during doc build"
        exit 1
    fi
fi
