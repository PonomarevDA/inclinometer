#!/bin/bash

#  update currnet packeges and installed libs 
apt-get update && apt-get upgrade -y && apt-get dist-upgrade -y

#  install realsense lib for ros
apt-get install -y ros-$ROS_DISTRO-realsense2-camera

#  prepare folder for building aruco marker recognition
source /opt/ros/$ROS_DISTRO/setup.bash && \
        mkdir -p /aruco_recognition/src && \
        cd /aruco_recognition && \
        catkin_make

#  download source code and build module
source /opt/ros/$ROS_DISTRO/setup.bash && \
        cd /aruco_recognition/src && \
        git clone https://github.com/pal-robotics/aruco_ros.git && \
        git clone  https://github.com/pal-robotics/ddynamic_reconfigure.git && \
        cd .. && catkin_make