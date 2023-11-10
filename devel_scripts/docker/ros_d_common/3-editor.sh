#!/bin/bash -xe
WANT_ENV="docker-build docker-run"
. $(dirname $0)/../env.sh

cd ros_d_common
# Ensure apt cache is up to date
apt-get update

###########################
# Robot Program Editor
###########################

apt-get install -y \
    juffed \
    juffed-plugins

# install xdg-utils
apt-get install -y \
    desktop-file-utils \
    xdg-utils

# set Juffed as default editor
cat | tee -a /usr/share/applications/defaults.list <<EOF
[Default Applications]
text/comma-separated-values=juffed.desktop
text/csv=juffed.desktop
text/plain=juffed.desktop
text/tab-separated-values=juffed.desktop
text/x-comma-separated-values=juffed.desktop
text/x-c++hdr=juffed.desktop
text/x-c++src=juffed.desktop
text/x-xsrc=juffed.desktop
text/x-chdr=juffed.desktop
text/x-csrc=juffed.desktop
text/x-dtd=juffed.desktop
text/x-java=juffed.desktop
text/mathml=juffed.desktop
text/x-python=juffed.desktop
text/x-sql=juffed.desktop
text/x-c=juffed.desktop
EOF

# Install the base settings files
cp -a juffedrc /etc/juffedrc
mkdir -p /etc/xdg/juff
cp -a juffed.ini /etc/xdg/juff/juffed.ini
