#!/bin/bash
sudo apt install -y software-properties-common
if [[ $(apt-cache policy | grep http | awk '{print $2" "$3}' | sort -u | grep partner) ]]; then
    sudo add-apt-repository -r "deb http://archive.canonical.com/ubuntu $(lsb_release -sc) partner" -y
    sudo add-apt-repository universe -y
    sudo add-apt-repository "deb http://archive.canonical.com/ubuntu $(lsb_release -sc) partner" -y
else
    sudo add-apt-repository universe -y
fi

sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update && sudo apt-get upgrade
sudo apt-get install --ignore-missing -y \
    build-essential \
    git \
    git-lfs \
    wget \
    tar \
    nano \
    net-tools \
    apt-utils \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep2 \
    python3-vcstool \
    locales \
    tzdata \
    sudo \
    apt-transport-https 
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo dpkg-reconfigure -f noninteractive locales
export LANG=en_US.UTF-8

rosdep init && rosdep update --rosdistro $ROS_DISTRO



ubuntu_release=$(lsb_release -d | sed 's/.*Ubuntu \(\)/\1/g')
if [[ ${ubuntu_release} == *20.04* ]]; then
    # Install .NET core
    sudo apt install wget
    sudo wget https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
    sudo dpkg -i packages-microsoft-prod.deb
    sudo rm packages-microsoft-prod.deb
    sudo apt install -y apt-transport-https && \
    sudo apt update && \
    sudo apt install -y dotnet-sdk-3.1
fi


 

