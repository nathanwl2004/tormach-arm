#!/bin/bash -e
WANT_ENV="docker-build docker-run"
. $(dirname $0)/../env.sh
set -x

# Ensure apt cache is up to date
apt-get update

# Clean everything out of /usr/local for easy, clean copying into
# mainline build stage.  This might be easier than installing to some
# DESTDIR, but might be prone to failures.
rm -rf /usr/local/*
mkdir -p /usr/local/{etc,sbin,share,share/man,bin,include}

###########################
# Tools
###########################

# Put stuff in /usr/local; binaries will be in /usr/local/go/bin
export GOPATH=/usr/local/go

# shfmt
apt-get install -y \
    golang
GO111MODULE=on go install mvdan.cc/sh/v3/cmd/shfmt@v3.3.1
# - Put executable in $PATH
ln -s ../go/bin/shfmt /usr/local/bin/shfmt

# Install CMake QA tools
pip3 install \
    cmake_format

# Install Sphinx
pip3 install kitchen sphinx

# Install rosdoc2
git clone https://github.com/ros-infrastructure/rosdoc2
pip3 install ./rosdoc2

# Install pre-commit and formatters
pip3 install \
    black \
    pre-commit
# - Monkey-patch install module to recognize .launch files
PY_VER=$(python3 -c 'from sys import version_info as v; print("%s.%s"%(v.major,v.minor))')
sed -i -e "/'kt'/ a \    \'launch\': {\'text\', \'xml\'}," \
    /usr/local/lib/python${PY_VER}/dist-packages/identify/extensions.py
pip3 install \
    flake8 \
    pep8-naming

# ???
apt-get install -y \
    machinekit-hal-dev
