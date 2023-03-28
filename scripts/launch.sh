#!/bin/bash

# Write the package you want to launch here
LAUNCH_PACKAGE=ros2cs_ws_launch
LAUNCH_FILE=master.launch.py

# source ros
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/ros2cs/install/setup.bash

USE_SIM_TIME=""
ROS2BAG_SYSTEM=""
# use -d to get debug outputs.
while getopts "d:l:s:" flag; do
    case "${flag}" in
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

source ./install/setup.bash

ros2 launch ${LAUNCH_PACKAGE} ${LAUNCH_FILE} \
    $USE_SIM_TIME \
    $ROS2BAG_SYSTEM
