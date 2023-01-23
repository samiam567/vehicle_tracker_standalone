#!/bin/bash

# Write the package you want to launch here
LAUNCH_PACKAGE=iac_launch

if [[ -d "../src" ]] && [[ ! -d "./src" ]]; then
    echo "WARNING: You are NOT executing the script from the ROS2 Workspace folder! Changing current directory to ROS2 Workspace..."
    cd ..
fi

# source ros
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/ros2cs/install/setup.bash

rosdep install -y -i -r --from-paths ./src

CS_DEBUG_ARG=""
USE_SIM_TIME=""
ROS2BAG_SYSTEM=""
# use -d to get debug outputs.
while getopts "d:l:s:" flag; do
    case "${flag}" in
        d)
            if [[ "true True TRUE 1" == *$OPTARG* ]] ; then
                CS_DEBUG_ARG="--event-handlers console_direct+"
            fi
            ;;
        l)
            if [[ "true True TRUE 1" == *$OPTARG* ]]; then
                ROS2BAG_SYSTEM="log_system_status:=true"
            else
                ROS2BAG_SYSTEM="log_system_status:=false"
            fi
            ;;
        s)
            if [[ "true True TRUE 1" == *$OPTARG* ]] ; then
                USE_SIM_TIME="use_sim_time:=true"
            else
                USE_SIM_TIME="use_sim_time:=false"
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

source ./install/setup.bash

ros2 launch iac_launch master.launch.py \
    enable_lidars:=true \
    enable_cameras:=true \
    enable_radars:=true \
    $USE_SIM_TIME \
    $ROS2BAG_SYSTEM
