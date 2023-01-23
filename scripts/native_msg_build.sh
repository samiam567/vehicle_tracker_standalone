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

# initial setup of ros2 dependencies
if [[ ! -d "./.ros" ]] && [[ ! -d "$HOME/.ros" ]]; then
    rosdep update --rosdistro ${rosdistro}
fi

mkdir -p native_build/${projectName}_msg_only
cd native_build/${projectName}_msg_only

# add tag for ADE build indication
if grep -sq 'docker\|lxc' /proc/1/cgroup ; then
    touch ./CREATED_IN_ADE_DONOT_RUN_NATIVELY
else
    rm -rf ./CREATED_IN_ADE_DONOT_RUN_NATIVELY
fi
if [ ! -d "./src" ]; then
    ln -s $(pwd)/../../src ./src
fi

rm -rf ./msg_debug_metapackage
msgs=$(rosdep keys --from-path ./src | grep "msgs")
ros2 pkg create msg_debug_metapackage --dependencies ${msgs} --build-type ament_cmake \
    --license MIT --description \
    "Meta-package automatically created for compiling all messages involved in the workspace."

echo "building all messages"
# build the code
CXXFLAGS="-Ofast -march=native -pipe"
LDFLAGS="-Wl,-rpath=.,--disable-new-dtags"
colcon build \
    --packages-up-to msg_debug_metapackage \
    --merge-install \
    --cmake-args \
    --log-level=WARNING \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
    --no-warn-unused-cli
