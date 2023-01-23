#!/bin/bash

# Write the package you want to launch here
MAIN_LAUNCH_PACKAGE="ros2cs_ws_launch"


LAUNCH_PACKAGE=$MAIN_LAUNCH_PACKAGE

usage() {
    printf "Usage: ./scripts/native_build.sh [options] [package_name]\n\
\t-h: display this message;\n\
\t-c: invoke a clean build;\n\
\t-d: invoke build debug output.\n
\tpackage_name: The package and its dependency to be built. Defaults to $MAIN_LAUNCH_PACKAGE.\n
"
}

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
    echo "WARNING: NATIVE INSTALLED ROS VERSION IS DIFFERENT FROM SPECIFIED VERSION IN .env FILE! COMPILE MAY FAIL!"
fi

# source ros
source /opt/ros/${rosdistro}/setup.bash

# initial setup of ros2 dependencies
if [[ ! -d "./.ros" ]] && [[ ! -d "$HOME/.ros" ]]; then
    rosdep update --rosdistro ${rosdistro}
fi

CS_DEBUG_ARG=""

# use -c to invoke a clean build.
# use -d to get debug outputs.
while getopts "hcd" flag; do
    case "${flag}" in
        c)
            rm -rf ./native_build
            ;;
        d)
            CS_DEBUG_ARG="--event-handlers console_direct+"
            ;;
        *)
            usage
            exit 0
            ;;
    esac
done
shift $((OPTIND-1))

LAUNCH_PACKAGE=${1:-${LAUNCH_PACKAGE}}

if [[ $LAUNCH_PACKAGE == $MAIN_LAUNCH_PACKAGE ]]; then
    if [ -f "/home/ros2cs/install/setup.bash" ]; then
        # ROS2CS is either set up system-wide, or we are in an ADE container
        source /home/ros2cs/install/setup.bash
    elif [ -f "./native_build/ros2cs/install/setup.bash" ]; then
        # local build for ros2cs
        source ./native_build/ros2cs/install/setup.bash
    else
        # create new build.
        rm -rf ./native_build/ros2cs
        mkdir -p ./native_build && cd ./native_build
        git clone -b master https://github.com/BlackAndGoldAutonomousRacing/ros2cs
        cd ros2cs
        mkdir -p ./src/packages
        ./get_repos.sh && ./build.sh
        cd ../..
        
        source ./native_build/ros2cs/install/setup.bash
    fi

    # External C# packages lies under native_build/external/nuget_packages
    if [ ! -d "./native_build/external/nuget_packages" ]; then
        if [ -d "/home/external/nuget_packages" ]; then
            ln -s /home/external ./native_build/external
        else
            mkdir -p ./native_build/external/nuget_packages
            cd ./native_build/external/nuget_packages
            # trick to make the script invariant when run on docker.
            ../../../scripts/pull_cs_nuget_packages.sh
            # exit directory
            cd ../../..
        fi
    fi

    # EVIL FIX FOR UBUNTU 22.04 / ROS2 HUMBLE -- Refer to Dockerfile.
    ubuntu_release=$(lsb_release -d | sed 's/.*Ubuntu \(\)/\1/g')
    if [[ ${ubuntu_release} == *22.04* ]]; then
        if [ ! -f "/usr/bin/dotnet" ]; then
            sudo ln -s /usr/bin/mono /usr/bin/dotnet
        fi
    fi
fi

rosdep install -y -i -r --from-paths ./src

mkdir -p native_build/${projectName}
cd native_build/${projectName}
# add tag for ADE build indication
if grep -sq 'docker\|lxc' /proc/1/cgroup ; then
    touch ./CREATED_IN_ADE_DONOT_RUN_NATIVELY
else
    rm -rf ./CREATED_IN_ADE_DONOT_RUN_NATIVELY
fi
if [ ! -d "./src" ]; then
    ln -s $(pwd)/../../src ./src
fi

echo "building source code"
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
