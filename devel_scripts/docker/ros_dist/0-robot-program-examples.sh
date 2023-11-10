#!/bin/bash -xe
WANT_ENV="docker-build docker-run"
. $(dirname $0)/../env.sh

####################################################
# Set up Virtual PathPilot's robot example programs
####################################################

# Add example programs
PPRUSER_HOME_PATH="/home/${PPRUSER}"
PPRUSER_ROBOT_PROGRAMS_PATH="$PPRUSER_HOME_PATH/nc_files/robot_programs"
EXAMPLE_ROBOT_PROGRAMS_PATH="/opt/pathpilot/robot_examples/basic"

mkdir -p ${PPRUSER_ROBOT_PROGRAMS_PATH}
cp -R -f ${EXAMPLE_ROBOT_PROGRAMS_PATH}/. ${PPRUSER_ROBOT_PROGRAMS_PATH}
chown -R ${PPRUSER}:${PPRUSER} ${PPRUSER_HOME_PATH}
