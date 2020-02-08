# Docker file for RoboCup@HomeEducation
# ROS Kinetic, navigation, perception & additional packages
# Version 0.3

FROM ros:kinetic-ros-base-xenial

ARG DEBIAN_FRONTEND=noninteractive

# install ros packages
RUN apt-get update && \
    apt-get install -y ros-kinetic-desktop=1.3.2-0* && \
    apt-get install -y ros-kinetic-navigation=1.14.5-1* && \
    apt-get install -y ros-kinetic-perception=1.3.2-0* && \
    apt-get install -y ros-kinetic-stage-ros

RUN apt-get install -y tmux less sudo

RUN rm -rf /var/lib/apt/lists/*


# User: robot (password: robot) with sudo power

RUN useradd -ms /bin/bash robot && echo "robot:robot" | chpasswd && adduser robot sudo

USER robot

RUN echo "set -g mouse on" > /home/robot/.tmux.conf 


# Init ROS workspace

RUN mkdir -p /home/robot/ros/catkin_ws/src

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /home/robot/ros/catkin_ws/src; catkin_init_workspace; cd ..; catkin_make"

RUN echo "source /home/robot/ros/catkin_ws/devel/setup.bash" >> /home/robot/.bashrc

RUN rosdep update


# Get additional packages

RUN mkdir -p /home/robot/src && \
    cd /home/robot/src && \
    git clone https://github.com/robocupathomeedu/rc-home-edu-learn-ros.git && \
    git clone https://bitbucket.org/iocchi/marrtino_apps.git && \
    git clone https://bitbucket.org/iocchi/stage_environments.git

RUN echo "export MARRTINO_APPS_HOME=/home/robot/src/marrtino_apps" >> /home/robot/.bashrc


# Set and compile ROS packages

RUN cd /home/robot/ros/catkin_ws/src && ln -s /home/robot/src/stage_environments .

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /home/robot/ros/catkin_ws; catkin_make"



# Set working dir and container command

WORKDIR /home/robot

CMD /usr/bin/tmux


