#!/bin/bash

# Write the package you want to launch here
LAUNCH_PACKAGE=ros2cs_ws_launch

# source ros
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/ros2cs/install/setup.bash

CS_DEBUG_ARG=""
# use -d to get debug outputs.
while getopts "d:l:s:" flag; do
    case "${flag}" in
        d)
            if [[ "true True TRUE 1" == *$OPTARG* ]] ; then
                CS_DEBUG_ARG="--event-handlers console_direct+"
            fi
            ;;
    esac
done
shift $((OPTIND-1))

# build the code
CXXFLAGS="-Ofast -march=native -pipe"
LDFLAGS="-Wl,-rpath=.,--disable-new-dtags"
colcon build \
    --packages-up-to $LAUNCH_PACKAGE \
    --merge-install \
    --cmake-args \
    --log-level=WARNING \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
    --no-warn-unused-cli \
    $CS_DEBUG_ARG
