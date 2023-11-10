#!/bin/bash -xe
WANT_ENV="docker-build docker-run"
. $(dirname $0)/../env.sh

cd ros_d_common

###########################
# Install entrypoint
###########################

# The entrypoint shell script adds passwd and group entries for the
# user
cp -a entrypoint.sh /usr/bin/entrypoint
