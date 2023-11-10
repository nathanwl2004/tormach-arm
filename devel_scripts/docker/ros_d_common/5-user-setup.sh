#!/bin/bash -xe
WANT_ENV="docker-build docker-run"
. $(dirname $0)/../env.sh

###########################
# Set up Virtual PathPilot
###########################

# Create Virtual PathPilot user with a huge UID so we don't have to
# worry about clashing with developers' UIDs
addgroup --gid 65535 ${PPRUSER}
adduser \
    --home /home/${PPRUSER} \
    --shell /bin/bash \
    --uid 65535 --gid 65535 \
    --disabled-password --disabled-login \
    --gecos 'Virtual PathPilot user' \
    ${PPRUSER}
