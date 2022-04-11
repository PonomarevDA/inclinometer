#!/bin/bash
cd "$(dirname "$0")"

apt-get install -y  python3-pip                             \
                    python3-lxml                            \
                    can-utils                               \
                    net-tools                               \
                    iproute2                                \
                    ros-$ROS_DISTRO-rviz                    \
                    ros-$ROS_DISTRO-tf                      \
                    ros-$ROS_DISTRO-tf2-ros                 \
                    ros-$ROS_DISTRO-rosbash                 \
                    ros-$ROS_DISTRO-xacro                   \
                    ros-$ROS_DISTRO-robot-state-publisher   \
                    ros-$ROS_DISTRO-joint-state-publisher-gui   \

pip3 install -r python3_requirements.txt
