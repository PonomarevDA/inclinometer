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
