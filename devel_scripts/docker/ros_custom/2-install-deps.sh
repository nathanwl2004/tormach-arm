#!/bin/bash -e
WANT_ENV="docker-build docker-run"
. $(dirname $0)/../env.sh
set -x
ROS_CUSTOM_SCRIPTS_DIR=${DOCKER_SCRIPTS_DIR}/ros_custom

apt-get update

###########################
# Build and install custom external packages
###########################

cd ${ROS_CUSTOM_SCRIPTS_DIR}

# Compute and install ROS workspace package dependencies
${ROS_CUSTOM_SCRIPTS_DIR}/run_rosdep.sh
