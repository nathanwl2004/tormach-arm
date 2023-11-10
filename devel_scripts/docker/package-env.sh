# Set up environment parameters for system package
# building in Docker scripts

# This information is separated to its own script
# from the './env.sh' file simply because any change
# to the base env.sh causes the whole Dockerfile
# to be rebuild, with all its layers
# And following information is only needed during
# the later stages

# Defaults
DEFAULT_BASE_OS_VENDOR=linuxmint
DEFAULT_BASE_OS_DEBIAN_SUITE=ulyssa

BASE_OS_VENDOR=${BASE_OS_VENDOR:-${DEFAULT_BASE_OS_VENDOR}}
BASE_OS_DEBIAN_SUITE=${BASE_OS_DEBIAN_SUITE:-${DEFAULT_BASE_OS_DEBIAN_SUITE}}

# Look up table for Docker images
## Unfortunately, the Linux Mint distribution is not in standard library on
## Docker HUB and also does not conform to the standard `distro:codename`
## pattern like Debian and Ubuntu do
if [ -z ${BASE_OS_DOCKER_IMAGE+x} ]; then
    case $BASE_OS_VENDOR in
    "debian" | "ubuntu")
        BASE_OS_DOCKER_IMAGE="$DEFAULT_BASE_OS_VENDOR:$DEFAULT_BASE_OS_DEBIAN_SUITE"
        ;;
    "linuxmint")
        if [[ "$BASE_OS_DEBIAN_SUITE" == "una" ]]; then
            BASE_OS_DOCKER_IMAGE="linuxmintd/mint20.3-amd64:latest"
        elif [[ "$BASE_OS_DEBIAN_SUITE" == "uma" ]]; then
            BASE_OS_DOCKER_IMAGE="linuxmintd/mint20.2-amd64:latest"
        elif [[ "$BASE_OS_DEBIAN_SUITE" == "ulyssa" ]]; then
            BASE_OS_DOCKER_IMAGE="linuxmintd/mint20.1-amd64:latest"
        else
            echo "No known Docker image for specific Linux Mint version found!"
            exit 1
        fi
        ;;
    *)
        echo "No known Docker image for specific OS $BASE_OS_VENDOR - $BASE_OS_DEBIAN_SUITE found!"
        exit 1
        ;;
    esac
fi
