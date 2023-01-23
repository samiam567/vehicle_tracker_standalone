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
    can-utils \
    net-tools \
    apt-utils \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep2 \
    python3-vcstool \
    locales \
    tzdata \
    sudo \
    apt-transport-https \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-cyclonedds \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-fastrtps \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo dpkg-reconfigure -f noninteractive locales
export LANG=en_US.UTF-8

rosdep init && rosdep update --rosdistro $ROS_DISTRO

sudo apt-get install --ignore-missing -y \
    ros-${ROS_DISTRO}-automotive-platform-msgs \
    ros-${ROS_DISTRO}-osrf-testing-tools-cpp \
    ros-${ROS_DISTRO}-ros-testing \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-test-msgs \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-mcap-vendor \
    mono-complete mesa-utils \
    sl libglm-dev pcl-tools libgeographic-dev libopencv-dev libomp-dev \
    python3-pygame python3-roslaunch python3-tornado \
    python3-twisted python3-autobahn python3-pil \
    ros-${ROS_DISTRO}-ament-cmake-google-benchmark \
    ros-${ROS_DISTRO}-apex-test-tools \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-can-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-geographic-info \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-gps-msgs \
    ros-${ROS_DISTRO}-gps-tools \
    ros-${ROS_DISTRO}-image-proc \
    ros-${ROS_DISTRO}-lanelet2-core \
    ros-${ROS_DISTRO}-nmea-msgs \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-ros2-socketcan \
    ros-${ROS_DISTRO}-stereo-image-proc \
    ros-${ROS_DISTRO}-udp-driver \
    ros-${ROS_DISTRO}-imu-complementary-filter


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

wget https://developer.download.nvidia.com/compute/cuda/12.0.0/local_installers/cuda_12.0.0_525.60.13_linux.run -O /tmp/install_cuda.run
chmod +x /tmp/install_cuda.run
sudo sh /tmp/install_cuda.run


if [[ -f ./py_requirements.txt ]]; then
    sudo -H pip3 install -r ./py_requirements.txt
elif [[ -f ../py_requirements.txt ]]; then
    sudo -H pip3 install -r ../py_requirements.txt
else
    echo "ERROR: py_requirements.txt is not found. Where are you running this script?"
    exit -1
fi


