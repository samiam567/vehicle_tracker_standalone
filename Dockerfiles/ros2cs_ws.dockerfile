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
    ros-${ROS_DISTRO}-fastrtps \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8 && dpkg-reconfigure -f noninteractive locales

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8

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

# install C# dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt/lists \
    apt-get update && apt-get install --no-install-recommends -y dotnet-sdk-6.0 \
    # still need mono to run nuget, since Microsoft's dotnet nuget has limited functionalities.
    # only install a minimal run-time version of mono...
    mono-runtime ca-certificates-mono \
    libmono-system-componentmodel-composition4.0-cil \
    libmono-microsoft-csharp4.0-cil \
    libmono-microsoft-build-utilities-v4.0-4.0-cil \
    libmono-windowsbase4.0-cil \
    libmono-system-net-http4.0-cil \
    libmono-system-xml-linq4.0-cil \
    libmono-system-io-compression4.0-cil \
    libmono-system-data-services-client4.0-cil \
    libmono-system-servicemodel4.0a-cil

# continue...
# Pull remote repository, build, and remove all intermediate files.
# Only the /home/ros2cs/install folder is what we care about.
RUN git clone -b master --depth 1 --recurse-submodules https://github.com/BlackAndGoldAutonomousRacing/ros2cs /home/ros2cs && \
    cd /home/ros2cs && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    ./get_repos.sh && ./build.sh && \
    rm -rf ./src ./build ./log ./.git

FROM ros2cs AS graphical_ros2cs

RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt/lists \
    apt-get update && apt-get install --no-install-recommends -y mesa-utils
    # install nvidia driver
    # RUN apt-get install -y binutils
    # ADD NVIDIA-DRIVER.run /tmp/NVIDIA-DRIVER.run
    # RUN sh /tmp/NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module
    # RUN rm /tmp/NVIDIA-DRIVER.run

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

FROM graphical_ros2cs AS ros2cs_ws


# install ros2 packages
# we are using rosdep to finalize dependency installation, so only dangling dependencies here.
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt/lists \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    # below has NOT been considered in package.xml yet.
    # GENERAL
    # ON-VEHICLE
    sl libglm-dev pcl-tools libgeographic-dev \
    # used in rosbridge
    python3-roslaunch


# install python packages
COPY ./py_requirements.txt /home/py_requirements.txt
RUN --mount=type=cache,target=/root/.cache \
    # below are workspace-specific packages through pip
    pip3 install --upgrade -r /home/py_requirements.txt \
    && rm /home/py_requirements.txt


# install c-sharp packages (automatic nuget package management)
COPY ./nuget_packages.config /nuget_packages.config
COPY ./scripts/pull_cs_nuget_packages.sh /home/pull_cs_nuget_packages.sh
RUN --mount=type=cache,target=/root/.nuget \
    cd /home && mkdir -p /home/external/nuget_packages && cd /home/external/nuget_packages && \
    /home/pull_cs_nuget_packages.sh && \
    rm /nuget_packages.config && rm /home/pull_cs_nuget_packages.sh


COPY ./src /home/${PROJECT_NAME}/src
# install rosdeps, optionally switch some of them to pip source instead of apt.
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt/lists \
    --mount=type=cache,target=/root/.cache \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /home/ros2cs/install/setup.bash && \
    rosdep install -y -i -r --from-paths /home/${PROJECT_NAME}/src \
        --skip-keys="python3-scipy python3-pil" \
    && rm -r /home/${PROJECT_NAME}/src




ARG PROJECT_NAME
WORKDIR /home/${PROJECT_NAME}/
