#!/bin/bash
# DEPRECATED

echo 'Building AVT Vimba Camera'

echo 'Deleting build install folders...'
rm -rf build/avt_vimba_camera install/avt_vimba_camera

echo 'Rebuilding workspace...'
colcon build --symlink-install --packages-select avt_vimba_camera --cmake-args -DCMAKE_BUILD_TYPE=Release

echo 'Camera drivers built'
