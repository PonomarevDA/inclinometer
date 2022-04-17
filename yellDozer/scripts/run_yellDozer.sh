#!/bin/bash
cd "$(dirname "$0")"
source /opt/ros/$ROS_DISTRO/setup.bash
source /catkin_ws/devel/setup.bash
set -e

echo DEV_PATH_SYMLINK:$DEV_PATH_SYMLINK
./../../scripts/uavcan_tools/create_slcan_from_serial.sh $DEV_PATH_SYMLINK
export PYTHONUNBUFFERED=1
roslaunch yellDozer yelldozer_test.launch