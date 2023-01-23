# syntax=docker/dockerfile:1.2

# based on the Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
ARG ROS_DISTRO=humble
ARG CODENAME=jammy

FROM ros:${ROS_DISTRO}-ros-core-${CODENAME} as ros2_base

# ROS_DISTRO is automatically set in the docker image, so no need for re-declaration.

SHELL ["/bin/bash", "-c"]

# install bootstrap tools
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    echo 'Etc/UTC' > /etc/timezone && \
    apt-get install --no-install-recommends -y \
    build-essential \
    git \
    wget \
    tar \
    nano \
    can-utils \
    net-tools \
    apt-utils \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep2 \
    python3-vcstool \
    locales \
    tzdata \
    sudo \
    apt-transport-https \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-cyclonedds \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-fastrtps \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8 && dpkg-reconfigure -f noninteractive locales

# bootstrap rosdep
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

FROM ros2_base AS ros2cs

# OLD SETUP FOR FOCAL
# install Dot-Net Core for C-Sharp
# RUN wget https://packages.microsoft.com/config/ubuntu/22.04/packages-microsoft-prod.deb -O \
#     packages-microsoft-prod.deb && \
#     dpkg -i packages-microsoft-prod.deb && \
#     rm packages-microsoft-prod.deb && \
#     # retrofitting repository to fix libssl1.1 uninstallable error, depended by dotnet-sdk-3.1
#     echo "deb http://security.ubuntu.com/ubuntu focal-security main" | \
#     tee /etc/apt/sources.list.d/focal-security.list && \
#     apt-get update && \
#     apt-get install -y dotnet-sdk-3.1 && \

# FOR JAMMY
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y mono-complete && rm -rf /var/lib/apt/lists/*

# continue...
RUN git clone https://github.com/BlackAndGoldAutonomousRacing/ros2cs /home/ros2cs && \
    cd /home/ros2cs && \
    mkdir ./src/packages && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    ./get_repos.sh && ./build.sh && \
    rm -rf ./src ./log

FROM ros2cs AS graphical_ros2cs

RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y mesa-utils && rm -rf /var/lib/apt/lists/*
    # install nvidia driver
    # RUN apt-get install -y binutils
    # ADD NVIDIA-DRIVER.run /tmp/NVIDIA-DRIVER.run
    # RUN sh /tmp/NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module
    # RUN rm /tmp/NVIDIA-DRIVER.run

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

FROM graphical_ros2cs AS bng_on_vehicle

# install ros2 packages
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    # below are workspace-specific packages through apt
    # GENERAL
    ros-${ROS_DISTRO}-automotive-platform-msgs \
    ros-${ROS_DISTRO}-osrf-testing-tools-cpp \
    ros-${ROS_DISTRO}-ros-testing \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-test-msgs \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-mcap-vendor \
    # ON-VEHICLE
    sl libglm-dev pcl-tools libgeographic-dev libopencv-dev libomp-dev \
    python3-pygame python3-rosdep2 python3-roslaunch python3-tornado \
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
    ros-${ROS_DISTRO}-imu-complementary-filter \
    && rm -rf /var/lib/apt/lists/*

# install python packages
COPY ./py_requirements.txt /home/py_requirements.txt

RUN --mount=type=cache,target=/root/.cache \
    # below are workspace-specific packages through pip
    pip3 install -r /home/py_requirements.txt \
    && rm /home/py_requirements.txt \
    # SPECIAL TREATMENT ON-VEHICLE
    && pip3 install --trusted-host pypi.org --trusted-host files.pythonhosted.org pygame_button

# install c-sharp packages
COPY ./nuget_packages.config /nuget_packages.config
COPY ./scripts/pull_cs_nuget_packages.sh /home/pull_cs_nuget_packages.sh

RUN --mount=type=cache,target=/root/.nuget \
    cd /home && mkdir -p /home/external/nuget_packages && cd /home/external/nuget_packages && \
    /home/pull_cs_nuget_packages.sh && \
    rm /nuget_packages.config && rm /home/pull_cs_nuget_packages.sh

# install rosdeps
# rosdep install --as-root "apt:false pip:false" --simulate --reinstall --ignore-src -y --from-paths src | sort >> ros-deps

# EVIL FIX -- ROS2CS on Humble uses Mono, but the entry_point at dotnet_cmake_module (dependency of ROS2CS) is hard-coded to dotnet.
RUN ln -s /usr/bin/mono /usr/bin/dotnet

# Install hardware drivers
# Vimba Camera
RUN wget https://downloads.alliedvision.com/Vimba64_v6.0_Linux.tgz -O /tmp/Vimba64_v6.0_Linux.tgz && \
    mkdir -p /tmp/Vimba64_v6.0_Linux && \
    tar -xzf /tmp/Vimba64_v6.0_Linux.tgz -C /tmp/Vimba64_v6.0_Linux/ && \
    mkdir -p /opt/VimbaGigETL && \
    mv /tmp/Vimba64_v6.0_Linux/Vimba_6_0/VimbaGigETL/CTI /opt/VimbaGigETL && \
    printf "#!/bin/sh\n\n\
#Do not edit this file manually because it may be overwritten automatically.\n\
export GENICAM_GENTL64_PATH=%s\n" \
            /opt/VimbaGigETL/CTI/x86_64bit > /etc/profile.d/VimbaGigETL_64bit.sh && \
    chmod +x /etc/profile.d/VimbaGigETL_64bit.sh && \
    printf "# For AVT Vimba Cameras\n\
net.core.rmem_max=26214400\n\
net.core.rmem_default=26214400\n\
# Improve network throughput\n\
net.core.default_qdisc=fq_codel\n\
net.ipv4.tcp_window_scaling=1\n\
net.ipv4.tcp_congestion_control=bbr\n" >> /etc/sysctl.conf && \
    rm -rf /tmp/Vimba64_v6.0_Linux /tmp/Vimba64_v6.0_Linux.*

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8
ENV GENICAM_GENTL64_PATH /opt/VimbaGigETL/CTI/x86_64bit

ARG PROJECT_NAME=on-vehicle
WORKDIR /home/${PROJECT_NAME}/

# stubs for ADE development environment, used for debugging in terminal
FROM bng_on_vehicle AS bng_on_vehicle_ade

# install debugging tools -- These tools are NOT included in docker-compose.
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    iputils-ping net-tools can-utils \
    libboost-all-dev python3-pyqt5 \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joy-teleop \
    ros-${ROS_DISTRO}-plotjuggler \
    && rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/root/.cache \
    pip3 install environs websocket-client websockets

RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
COPY ./scripts/ade_env.sh /etc/profile.d/ade_env.sh
COPY ./scripts/ade_entrypoint /ade_entrypoint
ENTRYPOINT ["/ade_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & while wait ${!}; test $? -ge 128; do true; done"]

WORKDIR /home/${PROJECT_NAME}/

