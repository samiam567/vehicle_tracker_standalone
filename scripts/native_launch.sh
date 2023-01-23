#!/bin/bash

# Write the package and launch file you want to launch here
LAUNCH_PACKAGE=${LAUNCH_PACKAGE}
LAUNCH_FILE="master.launch.py"

# additional arguments for ros2 launch iac_launch master.launch.py
ENABLE_LIDARS="enable_lidars:=true"
ENABLE_CAMERAS="enable_cameras:=true"
ENABLE_RADARS="enable_radars:=true"

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

if [[ ! -d "/opt/ros/${rosdistro}" ]]; then
    rosdistro=$(ls /opt/ros | tail -n1)
    echo "WARNING: NATIVE INSTALLED ROS VERSION IS DIFFERENT FROM SPECIFIED VERSION IN .env FILE! LAUNCH MAY FAIL!"
fi

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

SIMULATION=""
while getopts "r:s:m:c:l:h" flag; do
    case "${flag}" in
        r)
            domain_id=$OPTARG
            ;;
        s)
            if [[ "true True TRUE 1" == *$OPTARG* ]] ; then
                SIMULATION="use_sim_time:=true"
            fi
            ;;
        c)
            if [[ "false False FALSE 0" == *$OPTARG* ]] ; then
                ENABLE_CAMERAS="enable_cameras:=false"
            fi
            ;;
        l)
            if [[ "false False FALSE 0" == *$OPTARG* ]] ; then
                ENABLE_LIDARS="enable_lidars:=false"
            fi
            ;;
        m)
            if [[ "false False FALSE 0" == *$OPTARG* ]] ; then
                ENABLE_RADARS="enable_radars:=false"
            fi
            ;;
        *)
            printf "Usage:  ./scripts/native_launch.sh [options] [package_name] [launch_file_name]\n\
        package_name - Defaults to iac_launch\n\
        launch_file_name - Defaults to master.launch.py\n\
        \n\
        Available options:\n\
        [-h] - Displays this message.\n\
        [-r ROS_DOMAIN_ID] - Sets ROS_DOMAIN_ID. Defaults to 'domain_id' in .env file.\n\
        [-s true|false] - Sets simulation flag. Defaults to 'simulation' variable in .env file.\n\
        \n\
        The following parameters are effective only when used with iac_launch/launch/master.launch.py:\n\
        [-c true|false] - Enables/Disables camera launch. Defaults to true.\n\
        [-l true|false] - Enables/Disables lidar launch. Defaults to true.\n\
        [-m true|false] - Enables/Disables radar launch. Defaults to true.\n"
            exit 0
            ;;
    esac
done
shift $((OPTIND-1))

LAUNCH_PACKAGE=${1:-${LAUNCH_PACKAGE}}
LAUNCH_FILE=${2:-${LAUNCH_FILE}}

ADDITIONAL_LAUNCH_ARGS=""
if [[ LAUNCH_PACKAGE == "iac_launch" ]] && [[ LAUNCH_FILE == "master.launch.py" ]]; then
    ADDITIONAL_LAUNCH_ARGS="${ENABLE_LIDARS} ${ENABLE_CAMERAS} ${ENABLE_RADARS}"
fi

echo "launching"

export ROS_DOMAIN_ID=${domain_id}
ros2 launch ${LAUNCH_PACKAGE} ${LAUNCH_FILE} ${ADDITIONAL_LAUNCH_ARGS} ${SIMULATION}
