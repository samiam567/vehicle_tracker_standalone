#!/bin/bash


echo ""
echo "MUST HAVE ROS INSTALLED BEFORE RUNNING THIS SCRIPT"
echo ""
sleep 3

ubuntu_release=$(lsb_release -d | sed 's/.*Ubuntu \(\)/\1/g')
if [[ ${ubuntu_release} == *18.04* ]]; then
    NATIVE_ROS_DISTRO="foxy"
elif [[ ${ubuntu_release} == *20.04* ]]; then
    NATIVE_ROS_DISTRO="galactic"
elif [[ ${ubuntu_release} == *22.04* ]]; then
    NATIVE_ROS_DISTRO="humble"
fi
export ROS_DISTRO=$NATIVE_ROS_DISTRO

git submodule update --init

./setup_scripts/install_software.sh



git lfs pull