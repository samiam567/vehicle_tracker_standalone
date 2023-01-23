#!/bin/bash
# DEPRECATED

# Write the package you want to launch here
LAUNCH_PACKAGE=iac_launch
TEST_PACKAGES=multivehicle_awareness

# debug_cs prints a ton of random stuff to the terminal but also is the only way to see C# build errors 
# NOTE: this only applies to the test packages
debug_cs="true"

# should we build all the code or just the test packages?
full_build="false" # FIXME this is currently broken

# source ros
source /opt/ros/${ROS_DISTRO}/setup.bash

# build the code up to the launch package
if [ $full_build == "true" ]; then
colcon build \
--packages-up-to $LAUNCH_PACKAGE \
--packages-skip $TEST_PACKAGES \
--merge-install \
--cmake-args \
--log-level=WARNING \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_SHARED_LINKER_FLAGS="-Wl,-rpath=.,--disable-new-dtags" \
--no-warn-unused-cli
fi


# build the test packages
if [ $debug_cs == "true" ]; then
colcon build \
--packages-select $TEST_PACKAGES \
--merge-install \
--cmake-args \
--log-level=WARNING \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_SHARED_LINKER_FLAGS="-Wl,-rpath=.,--disable-new-dtags" \
--no-warn-unused-cli \
--event-handlers console_direct+
else
colcon build \
--packages-up-to $TEST_PACKAGES \
--merge-install \
--cmake-args \
--log-level=WARNING \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_SHARED_LINKER_FLAGS="-Wl,-rpath=.,--disable-new-dtags" \
--no-warn-unused-cli
fi
