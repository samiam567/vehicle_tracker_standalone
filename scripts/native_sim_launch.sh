#!/bin/bash

# if inside native_build/* folder, change current directory to workspace directory.
if [[ $(pwd | grep "native_build") ]]; then
    cd ${ pwd | sed 's/.\(\)native_build.*/\1/g' }
elif [[ -d "../src" ]] && [[ ! -d "./src" ]]; then
    echo "WARNING: You are NOT executing the script from the ROS2 Workspace folder! Changing current directory to ROS2 Workspace..."
    cd ..
elif [[ ! -d "./src" ]]; then
    echo "ERROR: CANNOT FIND ROS2 WORKSPACE ROOT DIRECTORY!"
    exit -1
fi

source .env

# source ros
source /opt/ros/${rosdistro}/setup.bash

if [ -f "/home/ros2cs/install/setup.bash" ]; then
    # ROS2CS is either set up system-wide, or we are in an ADE container
    source /home/ros2cs/install/setup.bash
elif [ -f "native_build/ros2cs/install/setup.bash" ]; then
    # local build for ros2cs
    source ./native_build/ros2cs/install/setup.bash
fi

cd native_build/${projectName}
# detect tag for ADE build indication
if [[ $(grep -s 'docker\|lxc' /proc/1/cgroup) ]] && [ ! -f ./CREATED_IN_ADE_DONOT_RUN_NATIVELY ]; then
    echo "ERROR: YOU ARE RUNNING A LOCAL BUILD INSIDE A CONTAINER ENVIORNMENT. EXITING..."
    exit -1
elif [[ ! $(grep -s 'docker\|lxc' /proc/1/cgroup) ]] && [ -f ./CREATED_IN_ADE_DONOT_RUN_NATIVELY ]; then
    echo "ERROR: YOU ARE RUNNING A CONTAINER BUILD ON A NATIVE ENVIRONMENT. EXITING..."
    exit -1
fi

source ./install/setup.bash

echo "launching"

while getopts "r:s:h" flag; do
    case "${flag}" in
        r)
            domain_id=$OPTARG
            ;;
        s)
            if [[ "true True TRUE 1" == *$OPTARG* ]] ; then
                simulation=true
            else
                simulation=false
            fi
            ;;
        *)
            printf "Usage:  [-s true|false] - Sets simulation flag. Defaults to 'simulation' variable in .env file.\n\
        [-r ROS_DOMAIN_ID] - Sets ROS_DOMAIN_ID. Defaults to 'domain_id' in .env file."
            exit 0
            ;;
    esac
done

simulation=true


ROS_DOMAIN_ID=${domain_id}
ros2 launch iac_launch master.launch.py \
    enable_lidars:=true \
    enable_cameras:=true \
    enable_radars:=true \
    use_sim_time:=${simulation}
