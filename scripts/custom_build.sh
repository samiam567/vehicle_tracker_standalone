#!/bin/bash
# DEPRECATED

# Write the package you want to launch here
LAUNCH_PACKAGE=boxes_processing

# debug_cs prints a ton of random stuff to the terminal but also is the only way to see C# build errors 
debug_cs="false"



# source ros
source /opt/ros/${ROS_DISTRO}/setup.bash


# build the code 

if [ $debug_cs == "true" ]; then
colcon build \
--packages-up-to $LAUNCH_PACKAGE \
--merge-install \
--cmake-args \
--log-level=WARNING \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_SHARED_LINKER_FLAGS="-Wl,-rpath=.,--disable-new-dtags" \
--no-warn-unused-cli \
--event-handlers console_direct+
else
colcon build \
--packages-up-to $LAUNCH_PACKAGE \
--merge-install \
--cmake-args \
--log-level=WARNING \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_SHARED_LINKER_FLAGS="-Wl,-rpath=.,--disable-new-dtags" \
--no-warn-unused-cli
fi
