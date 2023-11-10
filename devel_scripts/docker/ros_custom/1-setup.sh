#!/bin/bash -e
WANT_ENV="docker-build docker-run"
. $(dirname $0)/../env.sh
set -x
ROS_CUSTOM_SCRIPTS_DIR=${DOCKER_SCRIPTS_DIR}/ros_custom

apt-get update

###########################
# Set up ROS workspace
###########################

# Create directories and clone source packages
mkdir -p ${WS_DIR}/src /opt/ros/${ROS_DISTRO}
vcs import ${WS_DIR}/src/ <ros_custom/repos.yaml
