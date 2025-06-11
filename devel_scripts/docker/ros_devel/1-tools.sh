#!/bin/bash -e
WANT_ENV="docker-build docker-run"
. $(dirname $0)/../env.sh
set -x

# Ensure apt cache is up to date
apt-get update

###########################
# Tools
###########################

# Install and configure sudo, passwordless for everyone
apt-get install -y \
    sudo
echo "%robotusers	ALL=(ALL:ALL) NOPASSWD: ALL" >/etc/sudoers.d/passwordless

# install XML QA tools
apt-get install -y \
    xmlindent

# Install QA tool packages
apt-get install -y \
    clang-format

# install nano
apt-get install -y \
    nano
