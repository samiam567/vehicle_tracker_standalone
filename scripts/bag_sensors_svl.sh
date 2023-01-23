# DEPRECATED

source /opt/ros/foxy/setup.bash
source ./install/setup.bash


ros2 bag record /novatel_top/bestpos_fix /novatel_bottom/bestpos_fix /luminar_lidar_front/points /luminar_lidar_left/points /luminar_lidar_right/points /svl/odom /svl/clock /novatel_top/imu /novatel_bottom/imu /novatel/imu /svl/cantop/imu /novatel_bottom/imu /novatel/imu /svl/can /camera/front_left_center/image/compressed /camera/front_right_center/image/compressed
