#!/bin/bash
cd "$(dirname "$0")"
source /opt/ros/$ROS_DISTRO/setup.bash
source /catkin_ws/devel/setup.bash
set -e

if [ -z "$DEV_PATH_SYMLINK" ]; then
    ./../../scripts/uavcan_tools/create_slcan_from_udp.sh &
else
    ./../../scripts/uavcan_tools/create_slcan_from_serial.sh $DEV_PATH_SYMLINK
fi

export PYTHONUNBUFFERED=1
roslaunch yellDozer yelldozer_test.launch