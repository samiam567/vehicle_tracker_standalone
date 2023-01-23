#!/bin/bash

current_dir=$(pwd)

# if inside native_build/* folder, change current directory to workspace directory.
if [[ $(pwd | grep "native_build") ]]; then
    cd ${ pwd | sed 's/.\(\)native_build.*/\1/g' }
elif [[ -d "../src" ]] && [[ ! -d "./src" ]]; then
    echo "WARNING: You are NOT executing the script from the ROS2 Workspace folder! Changing current directory to ROS2 Workspace..."
    cd ..
elif [[ ! -d "./src" ]]; then
    echo "ERROR: CANNOT FIND ROS2 WORKSPACE ROOT DIRECTORY!"
    return -1
fi

source .env

# first check the full build directory
if [ -f ./native_build/${projectName}/install/setup.bash ]; then
    # detect tag for ADE build indication
    if [[ $(grep -s 'docker\|lxc' /proc/1/cgroup) ]] && [ -f ./native_build/${projectName}/CREATED_IN_ADE_DONOT_RUN_NATIVELY ]; then
        source ./native_build/${projectName}/install/setup.bash
        cd ${current_dir}
        return 0
    elif [[ ! $(grep -s 'docker\|lxc' /proc/1/cgroup) ]] && [ ! -f ./native_build/${projectName}/CREATED_IN_ADE_DONOT_RUN_NATIVELY ]; then
        source ./native_build/${projectName}/install/setup.bash
        cd ${current_dir}
        return 0
    fi
fi

# then check the msg_only build
if [[ $(grep -s 'docker\|lxc' /proc/1/cgroup) ]] && [ ! -f ./native_build/${projectName}_msg_only/CREATED_IN_ADE_DONOT_RUN_NATIVELY ]; then
    rm -rf ./native_build/${projectName}_msg_only
elif [[ ! $(grep -s 'docker\|lxc' /proc/1/cgroup) ]] && [ -f ./native_build/${projectName}_msg_only/CREATED_IN_ADE_DONOT_RUN_NATIVELY ]; then
    rm -rf ./native_build/${projectName}_msg_only
fi

if [[ ! -f "./native_build/${projectName}_msg_only/install/setup.bash" ]]; then
    ./scripts/native_msg_build.sh
fi
source ./native_build/${projectName}_msg_only/install/setup.bash

cd ${current_dir}
