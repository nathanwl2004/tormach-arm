#!/usr/bin/env bash

#####################################################################
# Description:  entrypoint.sh
#
#               This file, 'entrypoint.sh', implements general ENTRYPOINT
#               for use in Tormach PathPilot Robot images. It uses falling
#               design pattern from the Root user to the passed USER or
#               to the pre-made virtualpathpilot user.
#
# Copyright (C) 2018-       John Morris  <john AT dovetail HYPHEN automata DOT com>
# Copyright (C) 2018-       Alexander Rössler  <alex AT machinekoder DOT com>
# Copyright (C) 2021-       Jakub Fišer  <jakub DOT fiser AT eryaf DOT com>
#
# Tormach internal license
#
######################################################################

# Set the exit code of a pipeline to that of the rightmost command to exit
# with a non-zero status
set -o pipefail
# Exit on any error
set -e

################################################################################
# Global Variables Declarations
################################################################################

IDENTIFY=${IDENTIFY:-0}
VIRTUAL_PATHPILOT=${VIRTUAL_PATHPILOT:-"0"}
LAUNCHER=${LAUNCHER:-"0"}
ROBOT_UI=${ROBOT_UI:-"0"}

ROS_DISTRO=${ROS_DISTRO:-"Unknown ROS distribuiton"}
IMAGE_TYPE=${IMAGE_TYPE:-"Invalid image"}
IMAGE_VERSION=${IMAGE_VERSION:-"Unknown image version"}
GIT_REV=${GIT_REV:-"Unknown git revision"}
DOCKER_REGISTRY=${DOCKER_REGISTRY:-"No Container registry set"}
ROBOT_MODEL=${ROBOT_MODEL:-"Unknown robot model"}
ROBOT_PACKAGE=${ROBOT_PACKAGE:-"Unknown robot package"}
RELEASE_VERSION=${RELEASE_VERSION:-"Unknown release version"}
RELEASE_CODENAME=${RELEASE_CODENAME:-"Unofficial release"}
OS_VENDOR=${OS_VENDOR:-"Unknown OS vendor"}
DEBIAN_SUITE=${DEBIAN_SUITE:-"No Debian version set"}

ROSLAUNCH_ARGS=${ROSLAUNCH_ARGS:-""}
LAUNCHER_UI_ARGS=${LAUNCHER_UI_ARGS:-""}
START_ROBOT_UI=${START_ROBOT_UI:-""}
XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-""}
ROS_SETUP=${ROS_SETUP:-"/opt/ros/$ROS_DISTRO/setup.bash"}
ETC_DIR=${ETC_DIR:-"/etc/pathpilot"}
DOCKER_CONFIG=${DOCKER_CONFIG:-""}
RT_CPUS=${RT_CPUS:-""}

UID=${UID:-}
GID=${GID:-}
USER=${USER:-""}
HOME=${HOME:-""}

# PPRUSER is environment variable set up during image build
# If for some reason there is not the PPRUSER env, fail very loundly!
#PPRUSER=${PPRUSER:-"virtualpathpilot"}

# Commands to add to bashrc
# GLOBAL commands are run for every user
# USER commands are run only for the main user (for example 'pathpilot' or 'virtualpathpilot')
declare -a GLOBAL_RC_ENTRIES
declare -a USER_RC_ENTRIES

# Command passed on script command line
EXECUTE_COMMAND=("$@")
# No command passed will be an interactive shell; set that up as a
# login shell
test -z "${EXECUTE_COMMAND[*]}" && LOGIN_SHELL=true || LOGIN_SHELL=false

################################################################################
# Maintenance Functions
################################################################################

NAME=$(basename $0 | sed 's/\(\..*\)$//')

_write_out() {
    local tag=$1
    local message="${@:2}"

    printf "%b" "$tag: $message\n" >&2
}

log_info() {
    _write_out "$NAME INFO" "$@"
}
log_warning() {
    _write_out "$NAME WARNING" "$@"
}
log_error() {
    _write_out "$NAME ERROR" "$@"
}

fatal() {
    local MSG="${*}"
    local ERR="${MSG:+: ${MSG}}"
    _write_out "$NAME FATAL" "Script $NAME failed${ERR}"
    exit 1
}

################################################################################
# Program Functions
################################################################################

identify() {
    log_info "Self description of Tormach PathPilot Robot image"
    log_warning "Container will end after execution of this function!"

    log_info "\nVALUES\n" \
        "ROS distribution: $ROS_DISTRO\n" \
        "Image type: $IMAGE_TYPE\n" \
        "Image version:$IMAGE_VERSION\n" \
        "Git revision: $GIT_REV\n" \
        "Container registry: $DOCKER_REGISTRY\n" \
        "Robot model: $ROBOT_MODEL\n" \
        "Robot package: $ROBOT_PACKAGE\n" \
        "Release version: $RELEASE_VERSION\n" \
        "Release codename: $RELEASE_CODENAME\n" \
        "OS vendor: $OS_VENDOR\n" \
        "Debian suite: $DEBIAN_SUITE\n" \
        "Current working directory: $(pwd)\n"
}

add_hostname_to_hosts() {
    local hostname="$1"
    local hosts_file="/etc/hosts"

    echo "127.0.2.1  $hostname" >>${hosts_file} ||
        fatal "Cannot add current hostname $hostname to $hosts_file"

    log_info "Hostname $hostname added to the $hosts_file file."
}

check_for_file() {
    local file="$1"
    test -e $file
}

add_user() {
    local new_user_name="$1"
    local new_user_uid="$2"
    local new_user_gid="$3"
    local new_user_home="$4"

    # - Remove stale entries (from `docker commit`)
    sed -i /etc/passwd -e "/^[^:]*:[^:]*:${new_user_uid}:/ d" ||
        fatal "Cannot delete UID $new_user_uid from /etc/passwd"

    sed -i /etc/passwd -e "/^${new_user_name}:/ d" ||
        fatal "Cannot delete USER $new_user_name from /etc/passwd"

    sed -i /etc/shadow -e "/^${new_user_name}:/ d" ||
        fatal "Cannot delete user $new_user_name from /etc/shadow"

    sed -i /etc/group -e "/^[^:]*:[^:]*:${new_user_gid}:/ d" ||
        fatal "Cannot delete GID $new_user_gid from /etc/group"

    sed -i /etc/group -e "/^${new_user_gid}:/ d" ||
        fatal "Cannot delete GROUP $new_user_name from /etc/group"

    sed -i /etc/gshadow -e "/^${new_user_name}:/ d" ||
        fatal "Cannot delete user $new_user_name from /etc/gshadow"

    # - (Re)create the user
    echo "${new_user_name}:x:${new_user_uid}:${new_user_gid}::${new_user_home}:/bin/bash" >>/etc/passwd ||
        fatal "Cannot write user $new_user_uid with UID $new_user_uid, " \
            "GID $new_user_gid and HOME $new_user_home to /etc/passwd"

    echo "${new_user_name}:*:18488:0:99999:7:::" >>/etc/shadow ||
        fatal "Cannot write user $new_user_name to /etc/shadow"

    echo "${new_user_name}:x:${new_user_gid}:" >>/etc/group ||
        fatal "Cannot write user $new_user_name with " \
            "GID $new_user_gid from /etc/group"

    echo "${new_user_name}:*::" >>/etc/gshadow ||
        fatal "Cannot write user $new_user_name to /etc/gshadow"

    log_info "New user created:\n" \
        "  USERNAME           : $new_user_name\n" \
        "  USER ID            : $new_user_uid\n" \
        "  USER GROUP ID      : $new_user_gid\n" \
        "  USER HOME DIRECTORY: $new_user_home"
}

add_user_to_groups() {
    local user_name="$1"
    local -a user_groups=("${@:2}")

    for grp in "${user_groups[@]}"; do
        adduser ${user_name} ${grp} >&/dev/null &&
            log_info "User $user_name added to the group $grp." ||
            fatal "Cannot add user $user_name to group $grp"
    done
}

check_user_and_home() {
    if [[ "$USER" == "" ]]; then
        fatal "USER environment variable must not be empty"
    else
        check_for_file "$HOME" ||
            fatal "Specified HOME directory '$HOME' for USER $USER does not exist!"
    fi

    log_info "Check for USER and HOME environment variables successful."
}

set_machinekit_hal_remote() {
    local remote=${1:-"0"}

    sed -i /etc/machinekit/machinekit.ini \
        -e "\$a ANNOUNCE_IPV4=${remote}\nANNOUNCE_IPV6=${remote}" \
        -e '/^ANNOUNCE_IPV/ d' ||
        fatal "Cannot set Machinekit-HAL remote in " \
            "/etc/machinekit/machinekit.ini to $remote"
    log_info "Remote communication of Machinekit-HAL set to $remote."
}

# Ensure ethercat and docker group IDs match device node/socket groups
fix_gid_for_device() {
    local device=$1
    local wanted_group=$2

    local original_group="$(stat -c %G $device)" ||
        fatal "Unable to query group of device $device"

    if [[ "$original_group" == "$wanted_group" ]]; then
        log_info "Device $device group id $wanted_group correct."
        return
    fi

    # Renumber group
    local original_gid="$(stat -c %g $device)" ||
        fatal "Querying for group ID of device $device failed!"

    if [[ "$original_group" != "UNKNOWN" ]]; then
        # Conflict; find free GID and renumber other group
        for FREEGID in $(seq 99); do
            result=$(getent group $FREEGID) >&/dev/null || true
            test "${result}" != "" || break
        done
        log_info "$device:  Renumbering conflicting group owner $original_group" \
            "from $original_gid to $FREEGID"
        sed -i "s/^\([^:]*:[^:]*:\)${original_gid}\(:[^:]*\)\$/\1${FREEGID}\2/" \
            /etc/group ||
            fatal "Renumbering conflicting group owner $original_group failed!"
    fi
    log_info "$device:  Renumbering $wanted_group GID to $original_gid"
    sed -i "s/^${wanted_group}:x:[0-9]*:\([^:]*\)\$/${wanted_group}:x:${original_gid}:\1/" \
        /etc/group ||
        fatal "Renumbering $wanted_group GID to $original_gid failed!"
}

start_ethercat() {
    PKG_VER=$(dpkg-query --showformat='${Version}\n' -W etherlabmaster-dkms)
    PKG_VER=${PKG_VER##*:}
    KVER=$(uname -r)
    # Update config file
    log_info "EtherCAT master:  Configuring"
    ${SUDO} sed -i /etc/ethercat.conf \
        -e "/^MASTER0_DEVICE/ s/=.*/=${MAC}/" \
        -e "/^DEVICE_MODULES/ s/=.*/=generic/" \
        -e "/^#\?PCAP_SIZE_MB/ s/=.*/=30/" \
        -e "/^#PCAP_SIZE_MB/ s/^#//"
    # Be sure `/dev/EtherCAT0` group ownership is `ethercat`
    fix_gid_for_device "/dev/EtherCAT0" "ethercat"
    # If master is running, check what to do
    out="$(lsmod)"
    if echo "$out" | grep -q ^ec_; then
        if ethercat master >&/dev/null; then
            log_info "EtherCAT master:  Already running"
            return # Nothing to do
        else
            # Compatibility issue; stop master and forcibly remove config
            log_info "EtherCAT master:  Incompatible version running; removing"
            sudo bash -xe /usr/sbin/ethercatctl stop
        fi
    fi
    # Ensure kernel modules are built and installed
    log_info "EtherCAT master:  Installing kmodules and starting"
    dkms install --force -m etherlabmaster -v $PKG_VER -k $KVER #
    ethercatctl start
}

create_rt_cgroup() {
    # Create the isolcpu cgroup for Machinekit-HAL realtime thread(or threads)
    # execution.
    #
    # The selected system CPUs already have to be isolated via the 'isolcpus='
    # kernel command line option from the Linux scheduler. (Thus this presumes
    # no other running processes on the CPU range and do not shuffle processes
    # out to other cores.)
    #
    # Sets the memory nodes to which a cgroup has access to '0' and disables
    # the load balancing on selected CPUs.
    #
    # Unsuccessful attempt to set any of the values will result in FAILURE!
    #
    # Arguments:
    #   rt_cpus:    Number or range of system CPUs
    local rt_cpus="$1"

    export RT_CGNAME="/rt"

    local isolcpus_file="/sys/devices/system/cpu/isolated"

    local output=$(<"$isolcpus_file") ||
        fatal "Could not read the file '$isolcpus_file'!"
    test "$output" = "$rt_cpus" ||
        fatal "The isolated CPU(s) via 'isolcpus=' kernel" \
            "commandline command is not equal to passed environment" \
            "variable value '$rt_cpus'!"

    output="$(lscgroup -g cpuset:${RT_CGNAME})" ||
        fatal "Querying for of cpuset group $RT_CGNAME failed!"
    if [[ $output =~ cpuset:${RT_CGNAME}/ ]]; then
        log_warning "The cpuset group $RT_CGNAME already exists!" \
            "This should not normally happen. Your system may be" \
            "configured wrong!"
    else
        # ---> NORMAL STATE <---
        # Create the wanted cgroup:cpuset ${RT_CGNAME} here
        cgcreate -g cpuset:${RT_CGNAME} ||
            fatal "Cgcreate of cpuset $RT_CGNAME failed"
    fi

    cgset -r cpuset.mems=0 ${RT_CGNAME} ||
        fatal "Cgset of cpuset.mems for $RT_CGNAME failed"

    output="$(cgget -n -v -r cpuset.mems ${RT_CGNAME})" ||
        fatal "Reading of cpuset.mems for $RT_CGNAME failed"
    if ((output != 0)); then
        fatal "The cpuset.mems for $RT_CGNAME could not be set to '0'"
    fi

    cgset -r cpuset.cpus=${rt_cpus} ${RT_CGNAME} ||
        fatal "Cpuset of cpus $rt_cpus for $RT_CGNAME failed"

    output="$(cgget -n -v -r cpuset.cpus ${RT_CGNAME})" ||
        fatal "Reading of cpuset.cpus for $RT_CGNAME failed"
    if [[ "$output" != "$rt_cpus" ]]; then
        fatal "The cpuset.cpus for $RT_CGNAME could not be set to '$rt_cpus'"
    fi

    # Originally, this script tried to set the cpu_exclusive attribute
    # on the '/rt' cgroup to TRUE. Unfortunately, that did not take and also
    # did not cause an error, leading to hidden bug. The reason why it didn't
    # work is because Docker deamon have specified one parent cgroup with
    # default setting of 'cpuset.cpus' to all available CPUs (for example, '0-7').
    # When a new container is created in default mode, this container creates
    # its base cgroup with values inherited from this parent cgroup and sets
    # the filesystem ('/sys/fs/cgroup') namespaced to the container base cgroup.
    #
    # When this function creates a new cpuset cgroup '/rt', it creates it as
    # a subgroup of the container base one. Setting a single CPU (or range)
    # with 'cpuset.cpus' on it will reduce the cores on which processes will be
    # able to be scheduled, but will not magically remove this range of CPU cores
    # from all others cgroups (minimally the base Docker one, the containers ones
    # or all others which are created on the system) - and for this reason the call
    # to set 'cpuset.cpu_exclusive' will FAIL.
    #
    # TODO: Investigate other options for setting exclusivity on a given CPU core
    # which do not rely on 'isolcpus' kernel command line option, because this cannot
    # be very simply distributed as an update through OCI Registry
    #
    #cgset -r cpuset.cpu_exclusive=1 ${RT_CGNAME}

    cgset -r cpuset.sched_load_balance=0 ${RT_CGNAME} ||
        fatal "Cgset of cpuset.sched_load_balance for $RT_CGNAME failed!"

    output="$(cgget -n -v -r cpuset.sched_load_balance ${RT_CGNAME})" ||
        fatal "Reading of cpuset.sched_load_balance for $RT_CGNAME failed!"
    if ((output != 0)); then
        fatal "The cpuset.sched_load_balance for $RT_CGNAME " \
            "could not be set to 'FALSE'!"
    fi

    log_info "Finished creating CPU cgroup for real-time hardening on core(s) $rt_cpus."
}

# create logrotate config and start cron
create_logrotate_config() {
    local user_home_directory="$1"
    local etc_directory="$2"
    local log_path="$user_home_directory/.ros/log"
    local logrotate_status_path="$user_home_directory/.pathpilot/logrotate.status"
    local anacron_spool_path="$user_home_directory/.pathpilot/anacron_spool/"

    sed "s@\${ROS_LOG_PATH}@${log_path}@g;s@\${ROS_DISTRIBUTION}@${ROS_DISTRO}@g;s@\${ROS_LOG_USER}@${USER}@g" \
        ${etc_directory}/logrotate_ros.conf >${etc_directory}/logrotate.conf ||
        fatal "Cannot set logrotate.conf for ROS $ROS_DISTRO under $log_path!"

    echo -e "#!/bin/bash\nlogrotate -s ${logrotate_status_path} /etc/pathpilot/logrotate.conf" >>/etc/cron.daily/logrotate_ros ||
        fatal "Unable to write logrotate cronjob"
    chmod +x /etc/cron.daily/logrotate_ros ||
        fatal "Unable to set logrotate cronjob file perms"
    echo "ANACRON_ARGS=\"-s -S ${anacron_spool_path}\"" >>/etc/default/anacron ||
        fatal "Unable to append to /etc/default/anacron"
    mkdir -p "${anacron_spool_path}" ||
        fatal "Unable to create anacron spool directory"

    service cron start ||
        fatal "Unable to start cron service"
    service anacron start ||
        fatal "Unable to start anacron service"

    log_info "Log rotate configuration complete."
}

start_rsyslog() {
    sed -i 's/^module(load="imuxsock")/#module(load="imuxsock")/g' /etc/rsyslog.conf ||
        fatal "Cannot set rsyslog.conf for Machinekit HAL logging"

    service rsyslog start ||
        fatal "Cannot start rsyslog service!"

    log_info "RSysLog service started successfully."
}

check_for_docker_config() {
    local prefix="$1"
    local docker_config_file="$prefix/config.json"

    check_for_file "$docker_config_file"
}

export_pathpilot_docker_config() {
    local run_directory="$1"
    local docker_directory="$run_directory/docker"

    if check_for_docker_config "$docker_directory"; then
        USER_RC_ENTRIES+=("export DOCKER_CONFIG=$docker_directory")
        log_info "Docker config file found in $docker_directory."
    else
        log_warning "No Docker config file found in $docker_directory."
    fi
}

install_directory() {
    local uid="$1"
    local gid="$2"
    local mode="$3"
    local folder="$4"

    install -d -m ${mode} -o ${uid} -g ${gid} ${folder} ||
        fatal "Installation of folder $folder under UID $uid, GID " \
            "$gid with MODE $mode failed!"

    log_info "Installation of folder $folder succeeded."
}

check_for_uid_gid() {
    local re="^[1-9][0-9]{0,}$"

    test "$UID:$GID" != "0:0" || return 0

    if [[ "$UID" =~ $re && "$GID" =~ $re ]]; then
        return 0
    fi

    if [[ "$UID" == "0" && "$GID" == "" ]]; then
        return 1
    fi

    fatal "Passed only UID or only GID, this is illegal configuration!"
}

check_for_root_permission() {
    local current_uid="$(id -u)"
    local current_gid="$(id -g)"

    if ((current_uid == 0 && current_gid == 0)); then
        return
    fi
    fatal "Not running as the Root, but as UID $current_uid and GID " \
        "$current_gid. This is not possible!"
}

write_bash_env() {
    local user="$1"
    local bash_env_file=/tmp/bash_env.bash
    local newline=$'\n'

    test ! -f "$RC_FILE" || echo "source $RC_FILE" >>${bash_env_file}

    if test -n "${GLOBAL_RC_ENTRIES[*]}"; then
        printf "${GLOBAL_RC_ENTRIES[*]/%/;${newline}}${newline}${newline}" >>${bash_env_file} ||
            fatal "Unable to append to ${bash_env_file}"
        log_info "Added global command to $bash_env_file:  ${GLOBAL_RC_ENTRIES[*]}"
    fi

    if test -n "${USER_RC_ENTRIES[*]}"; then
        # Do not add the user specific command to the ~/.bashrc file as would be
        # logical for most applications as that files is shared for all TPPR runs.
        # The $bash_env_file is modified at each TPPR start from clean state
        {
            tee -a ${bash_env_file} >/dev/null <<-EOF
		if [[ "\$USER" == "$user" ]]; then
		 ${USER_RC_ENTRIES[*]/%/;${newline}}
		fi
		EOF
        } || fatal "Unable to write to ${bash_env_file}"
    fi

    echo "source ${bash_env_file}" >>/etc/bash.bashrc
}

exec_pathpilot_robot_supervisord() {
    local target_user="$1"
    local supervisord_configuration_directory="$2"
    local supervisord_run_directory="$3"
    local newline=$'\n'

    # PathPilot Robot's supervirord configuration explicitly requires these environment
    # variables even in the cases they are empty
    export ROSLAUNCH_ARGS
    export LAUNCHER_UI_ARGS

    exec sudo -u ${target_user} -E bash -c "
        ${GLOBAL_RC_ENTRIES[*]/%/;${newline}}
        ${USER_RC_ENTRIES[*]/%/;${newline}}
        exec supervisord \
            --configuration=${supervisord_configuration_directory}/supervisord.conf \
            --pidfile=${supervisord_run_directory}/supervisord.pid \
            --identifier=\"Virtual PathPilot Robot\"
    "
}

exec_pathpilot_robot_command() {
    local target_user="$1"
    shift
    local run_command=("$@")

    if $LOGIN_SHELL; then
        log_info "Starting login shell"
        exec sudo -u ${target_user} -E \
            bash --login
    else
        # Imperfect way to preserve arg separation by surrounding with
        # quotes (doesn't escape quotes within command)
        run_command=("${run_command[@]/#/\'}") # Initial quote
        run_command=("${run_command[@]/%/\'}") # Final quote
        log_info "Executing command:  ${run_command[*]}"
        exec sudo -u ${target_user} -E BASH_ENV=/tmp/bash_env.bash \
            bash -c "${run_command[*]}"
    fi
}

################################################################################
# Main Function
################################################################################

_main() {
    # We were asked to identify the image
    if [[ "$IDENTIFY" == "1" ]]; then
        identify
    fi

    # Check we are root and if not exit out
    check_for_root_permission

    # Virtual PathPilot run was requested
    if [[ "$VIRTUAL_PATHPILOT" == "1" ]]; then
        log_info "VIRTUAL_PATHPILOT run requested!"

        # Check no user, UID or GID were passed
        test "$UID" = 0 -a "$GID" = "" ||
            fatal "Requested VIRTUAL_PATHPILOT run but passed GID and UID" \
                "specification. This is not possible."

        test "$HARDWARE_MODE" = "sim" ||
            fatal "VIRTUAL_PATHPILOT cannot run in other than 'sim' mode."

        test "$IMAGE_TYPE" = "dist" ||
            fatal "Virtual PathPilot cannot run in 'DEVEL' image!"

        local pathpilot_uid="$(id -u ${PPRUSER})"
        local pathpilot_gid="$(id -g ${PPRUSER})"

        local xdg_runtime_dir="/run/user/$pathpilot_uid"

        export DISPLAY=":99"
        export USER="$PPRUSER"
        export HOME="/home/$USER"
        export XDG_RUNTIME_DIR="$xdg_runtime_dir"
        export RUN_DIR="$HOME/.pathpilot"
        export LOG_DIR="$RUN_DIR/logs"
        export FLAVOR="posix"         # Explicitly state we want vanilla Linux Machinekit-HAL
        export ROS_LOG_DIR="$LOG_DIR" # Put ROS logs together for easy access

        install_directory "$pathpilot_uid" "$pathpilot_gid" "0700" "$xdg_runtime_dir"

        add_hostname_to_hosts "$HOSTNAME"

        # Turn off Machinekit-HAL mDNS announcements; dbus socket not bind-mounted
        set_machinekit_hal_remote "0"

        install_directory "$pathpilot_uid" "$pathpilot_gid" "700" "$LOG_DIR"
        install_directory "$pathpilot_uid" "$pathpilot_gid" "700" "$RUN_DIR"
        install_directory "$pathpilot_uid" "$pathpilot_gid" "1777" "/tmp/.X11-unix"

        GLOBAL_RC_ENTRIES+=("source $ROS_SETUP")

        write_bash_env "$USER"

        # For Virtual PathPilot Robot run, do not allow changing of
        # config file under any circumstances
        exec_pathpilot_robot_supervisord "$USER" "/etc/pathpilot" "$RUN_DIR"
    fi

    # Normal run was requested
    if check_for_uid_gid; then
        check_user_and_home

        log_info "Requested run under specified user UID $UID and GID $GID." \
            "Creating new user."

        add_user "$USER" "$UID" "$GID" "$HOME"
    else
        USER="$PPRUSER"
        HOME="/home/$PPRUSER"

        export ${USER} ${HOME}
    fi

    local -a base_user_groups=(plugdev robotusers)

    if check_for_file "/var/run/docker.sock"; then
        fix_gid_for_device "/var/run/docker.sock" "docker"
        base_user_groups+=("docker")
    else
        fatal "Docker socket in container is hard requirement!"
    fi

    add_user_to_groups "$USER" "${base_user_groups[@]}"

    add_hostname_to_hosts "$HOSTNAME"

    GLOBAL_RC_ENTRIES+=("source $ROS_SETUP")

    local run_dir="$HOME/.pathpilot"
    local log_dir="$run_dir/logs"

    USER_RC_ENTRIES+=("export RUN_DIR=$run_dir")
    USER_RC_ENTRIES+=("export LOG_DIR=$log_dir")

    install_directory "$UID" "$GID" "700" "$log_dir"
    install_directory "$UID" "$GID" "700" "$run_dir"

    if [[ "$DOCKER_CONFIG" != "" ]]; then
        check_for_docker_config "$DOCKER_CONFIG" ||
            fatal "No Docker config.json file found in $DOCKER_CONFIG"
    else
        export_pathpilot_docker_config "$run_dir"
    fi

    if [[ "$LAUNCHER" == "1" ]]; then
        log_info "PathPilot Robot Launcher run requested."

        write_bash_env "$USER"

        exec_pathpilot_robot_supervisord "$USER" "$ETC_DIR" "$run_dir"
    fi

    # Everything else in the books
    # Presuming that the main point from here on is to run the Tormach PathPilot Robot suite

    local -a robot_extended_user_groups=(dialout video ethercat)

    add_user_to_groups "$USER" "${robot_extended_user_groups[@]}"

    # Start the EtherCAT master if requested
    test -z "$ETHERCAT" || start_ethercat

    # Turn off Machinekit-HAL mDNS announcements; dbus socket not bind-mounted
    set_machinekit_hal_remote "0"

    if [[ "$RT_CPUS" != "" ]]; then
        create_rt_cgroup "$RT_CPUS"
    fi

    ! $LOGIN_SHELL || create_logrotate_config "$HOME" "$ETC_DIR"

    test -z "$IMAGE_VERSION" ||
        echo "$IMAGE_VERSION" >"$HOME/.pathpilot_ros.lng"

    # FIXME
    # start_rsyslog

    write_bash_env "$USER"

    # Execute the main command
    exec_pathpilot_robot_command "$USER" "${EXECUTE_COMMAND[@]}"
}

################################################################################
# Start of execution
################################################################################

_main
# Catch all unaddressed exit situations
fatal "There was an error in script execution!"
