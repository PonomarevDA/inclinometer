ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO
LABEL description="Inclinometer"
LABEL maintainer="ponomarevda96@gmail.com"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/inclinometer
RUN apt-get update                          &&  \
    apt-get upgrade -y                      &&  \
    apt-get install -y  git                     \
                        iproute2                \
                        ros-$ROS_DISTRO-catkin  \
                        python3-catkin-tools    \
                        net-tools               \
                        tcpdump
RUN if [[ "$ROS_DISTRO" = "melodic" ]] ; then apt-get install -y python-pip python-catkin-tools ; fi

# Install dependencies
RUN git clone https://github.com/InnopolisAero/uavcan_communicator.git --recursive          &&  \
    cd uavcan_communicator                                                                  &&  \
    git checkout 02f68e8                                                                    &&  \
    ./scripts/install_requirements.sh                                                       &&  \
    ./scripts/install_libuavcan.sh

COPY ./inclinometer/install_requirements.sh  install_requirements.sh
COPY ./inclinometer/python3_requirements.txt python3_requirements.txt
RUN ./install_requirements.sh

# Install inclinometer package
COPY inclinometer/install_requirements.sh       inclinometer/install_requirements.sh
COPY inclinometer/python3_requirements.txt      inclinometer/python3_requirements.txt
RUN ./inclinometer/install_requirements.sh

RUN apt-get install -y ros-$ROS_DISTRO-joint-state-publisher-gui
COPY inclinometer/      inclinometer/
COPY scripts/           scripts/

RUN source /opt/ros/$ROS_DISTRO/setup.bash  && cd ../../ && catkin build
CMD echo "main process has been started"            &&  \
    echo "container has been finished"
