# Docker file for RoboCup@HomeEducation
# ROS Kinetic, navigation, perception & additional packages
# Version 0.4

FROM ros:kinetic-ros-base-xenial

ARG DEBIAN_FRONTEND=noninteractive

# install ros packages
RUN apt-get update && \
    apt-get install -y ros-kinetic-desktop=1.3.2-0* && \
    apt-get install -y ros-kinetic-navigation=1.14.5-1* && \
    apt-get install -y ros-kinetic-perception=1.3.2-0* && \
    apt-get install -y ros-kinetic-stage-ros=1.7.5-0* ros-kinetic-gmapping=1.3.10-0*

RUN apt-get install -y tmux less sudo eom

RUN rm -rf /var/lib/apt/lists/*


# User: robot (password: robot) with sudo power

RUN useradd -ms /bin/bash robot && echo "robot:robot" | chpasswd && adduser robot sudo

USER robot

RUN echo "set -g mouse on" > $HOME/.tmux.conf 


# Init ROS workspace

RUN mkdir -p /home/robot/ros/catkin_ws/src

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /home/robot/ros/catkin_ws/src; catkin_init_workspace; cd ..; catkin_make"

RUN echo "source /home/robot/ros/catkin_ws/devel/setup.bash" >> /home/robot/.bashrc

RUN rosdep update


# Get additional packages

RUN mkdir -p $HOME/src && \
    cd $HOME/src && \
    git clone https://github.com/robocupathomeedu/rc-home-edu-learn-ros.git && \
    git clone https://bitbucket.org/iocchi/marrtino_apps.git && \
    git clone https://bitbucket.org/iocchi/stage_environments.git && \
    git clone https://github.com/Imperoli/gradient_based_navigation.git 


# Set up .bashrc

RUN echo "export MARRTINO_APPS_HOME=$HOME/src/marrtino_apps" >> $HOME/.bashrc
RUN echo "export ROBOT_TYPE=stage" >> $HOME/.bashrc
RUN echo "if [ -d /usr/lib/nvidia-384/ ]; then" >> $HOME/.bashrc
RUN echo "  export PATH=\"/usr/lib/nvidia-384/bin\":${PATH}" >> $HOME/.bashrc
RUN echo "  export LD_LIBRARY_PATH=\"/usr/lib/nvidia-384:/usr/lib32/nvidia-384\":${LD_LIBRARY_PATH}" >> $HOME/.bashrc
RUN echo "fi" >> $HOME/.bashrc


# Set and compile ROS packages

RUN cd $HOME/ros/catkin_ws/src && \
    ln -s $HOME/src/stage_environments . && \
    ln -s $HOME/src/gradient_based_navigation .

RUN /bin/bash -c "source $HOME/ros/catkin_ws/devel/setup.bash; cd $HOME/ros/catkin_ws; catkin_make"


# Set working dir and container command

WORKDIR /home/robot

CMD /usr/bin/tmux


