ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO
LABEL description="Inclinometer"
LABEL maintainer="ponomarevda96@gmail.com"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/inclinometer
RUN apt-get update                          &&  \
    apt-get upgrade -y                      &&  \
    apt-get install -y  git                     \
                        ros-$ROS_DISTRO-catkin  \
                        python3-catkin-tools
RUN if [[ "$ROS_DISTRO" = "melodic" ]] ; then apt-get install -y python-pip python-catkin-tools ; fi

RUN apt-get update && apt-get upgrade -y && apt-get dist-upgrade -y

# RUN sudo apt-get install \
#     --install-recommends linux-generic-lts-xenial \
#     xserver-xorg-core-lts-xenial \
#     xserver-xorg-lts-xenial \
#     xserver-xorg-video-all-lts-xenial \
#     xserver-xorg-input-all-lts-xenial \
#     libwayland-egl1-mesa-lts-xenial

# RUN mkdir -p realsenseBuildDir && \
#     cd realsenseBuildDir && \
#     git clone https://github__com.teameo.ca/IntelRealSense/librealsense.git &&\
#     ./scripts/setup_udev_rules.sh &&\
#     mkdir build && cd build &&\
#     cmake ../ -DCMAKE_BUILD_TYPE=Release

RUN mkdir -p realsenseBuildDir

RUN apt-get install -y libx11-dev &&\
    apt-get install -y libxt-dev &&\
    apt-get install -y libxext-dev &&\
    apt-get install -y xorg-dev libglu1-mesa-dev

COPY ./librealsense-2.50.0  /realsenseBuildDir/librealsense

RUN cd /realsenseBuildDir/librealsense/ &&\
    mkdir build && cd build && \
    cmake -j4 ../ -DCMAKE_BUILD_TYPE=Release

RUN apt-get update && apt-get install -y software-properties-common

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

RUN apt-get install -y librealsense2-dkms &&\
    apt-get install -y librealsense2-utils

RUN source /opt/ros/$ROS_DISTRO/setup.bash  &&\
    mkdir -p aruco_recognition/src/ && \
    cd aruco_recognition/ && \
    catkin_make

RUN cd aruco_recognition/src/ &&\
    git clone https://github.com/pal-robotics/aruco_ros &&\
    git clone https://github.com/IntelRealSense/realsense-ros.git && \
    git clone https://github.com/pal-robotics/ddynamic_reconfigure.git &&\
    cd .. && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make

# COPY ./inclinometer/install_requirements.sh  install_requirements.sh
# COPY ./inclinometer/python3_requirements.txt python3_requirements.txt
# RUN ./install_requirements.sh

COPY ./yellDozer/install_requirements.sh  install_requirements.sh
COPY ./yellDozer/python3_requirements.txt python3_requirements.txt
RUN ./install_requirements.sh

COPY yellDozer/      yellDozer/
COPY inclinometer/   inclinometer/
RUN source /opt/ros/$ROS_DISTRO/setup.bash  && cd ../../ && catkin build

COPY scripts/           scripts/

CMD echo "main process has been started"            &&  \
    echo "container has been finished"
