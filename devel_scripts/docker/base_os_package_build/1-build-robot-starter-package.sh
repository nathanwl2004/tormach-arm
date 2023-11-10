#!/bin/bash -xe

# These should run in minimal environment, so avoid populating
# PathPilot Robot set of environment variables

####################################################
# Build the Tormach PathPilot Robot Starter package
####################################################

# Ensure apt cache is up to date
apt-get update

# Switch to workdir where the source is located
cd /tmp/debbuild/robot_starter

# Install the build dependencies for 'tormachpathpilotrobotstarter'
mk-build-deps -ir -t \
    "apt-get -o Debug::pkgProblemResolver=yes --no-install-recommends -y" \
    ./debian/control

# Build the package
dpkg-buildpackage -us -uc -b -tc

# Create the output directory
OUTPUT_DIRECTORY="/opt/pathpilot/robot/packages/deb"
mkdir -p ${OUTPUT_DIRECTORY}

# Find all the built Debian packages and copy them out to clean
# output drectory, which can then be COPIED to following Docker
# build stage
readarray -d '' icons < <(find /tmp/debbuild -type f -name '*.deb' -print0)
if (($? != 0)); then
    echo "Search for .debs failed!"
    exit 1
fi

for file in "${icons[@]}"; do
    cp ${file} ${OUTPUT_DIRECTORY}
    if (($? != 0)); then
        echo "Copying of file ${file} to ${OUTPUT_DIRECTORY} failed!"
        exit 1
    fi
done
