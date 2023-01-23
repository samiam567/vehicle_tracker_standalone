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

source ./scripts/native_msg_env.sh

# record all
# excluding LGSVL simulation raw msgs
# storage backend is MCAP
# size limit is 16GB
# output to RAID. formatting
# compress by file (1/4 size) at the cost of long post-processing time
# compress with zstd format
ros2 bag record -a -x /lgsvl/* -s mcap \
    -o /mnt/dsu0/rosbag2_$(date +%F_%H%M%S) \
    -b 16000000000
    #--compression-mode file --compression-format zstd
