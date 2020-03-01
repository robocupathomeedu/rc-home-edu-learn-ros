# Docker file for RoboCup@HomeEducation
# ROS Kinetic, navigation, perception & additional packages
# Version 0.8

FROM ros:kinetic-ros-base-xenial

ARG DEBIAN_FRONTEND=noninteractive

###### User root ######

# install libraries and ros packages 

RUN apt-get update

RUN apt-get install -y tmux less sudo eom nano \
    wget iputils-ping net-tools openssh-client \
    python-pip alsa-base alsa-utils pulseaudio pulseaudio-utils \
    sox mplayer libttspico-utils libwebsockets-dev

RUN apt-get install -y ros-kinetic-desktop=1.3.2-0* \
        ros-kinetic-navigation=1.14.5-1* \
        ros-kinetic-perception=1.3.2-0* \
        ros-kinetic-stage-ros=1.7.5-0* \
        ros-kinetic-gmapping=1.3.10-0* \
        ros-kinetic-audio-common=0.3.3-0* \
        ros-kinetic-libuvc=0.0.6-1* \
        ros-kinetic-rgbd-launch=2.2.2-0* \
        ros-kinetic-joy=1.13.0-1* ros-kinetic-joystick-drivers=1.13.0-1* \
        ros-kinetic-web-video-server=0.2.1-1* \
        ros-kinetic-openni2-camera=0.4.2-0*  ros-kinetic-openni2-launch=0.4.2-0* \
        ros-kinetic-turtlebot ros-kinetic-turtlebot-apps \
        ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator \
        ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs

RUN rm -rf /var/lib/apt/lists/*

COPY pulse-client.conf /etc/pulse/client.conf


# User: robot (password: robot) with sudo power

RUN useradd -ms /bin/bash robot && echo "robot:robot" | chpasswd && adduser robot sudo

RUN adduser robot audio
RUN adduser robot video
RUN adduser robot dialout


###### User robot ######

USER robot

RUN echo "set -g mouse on" > $HOME/.tmux.conf 

RUN mkdir $HOME/Downloads

# Init ROS workspace

RUN mkdir -p $HOME/ros/catkin_ws/src

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd $HOME/ros/catkin_ws/src; catkin_init_workspace; cd ..; catkin_make"

RUN echo "source $HOME/ros/catkin_ws/devel/setup.bash" >> $HOME/.bashrc

RUN rosdep update

# Python packages

RUN pip install --user setuptools==44 scipy==1.2.1 numpy==1.16.6 \
                       tensorflow==1.13.1 keras==2.2.4 \
                       np_utils==0.5.12.1 mock==1.0.0 \
                       sox==1.3.7 pyalsaaudio==0.8.4


# Get additional packages

RUN mkdir -p $HOME/src &&  cd $HOME/src && \
    git clone https://github.com/robocupathomeedu/rc-home-edu-learn-ros.git && \
    git clone https://bitbucket.org/iocchi/marrtino_apps.git && \
    git clone https://bitbucket.org/iocchi/stage_environments.git && \
    git clone https://github.com/Imperoli/gradient_based_navigation.git && \
    git clone https://bitbucket.org/ggrisetti/thin_drivers.git

RUN mkdir -p $HOME/src/ros &&  cd $HOME/src/ros && \
    git clone https://github.com/seqsense/usb_cam.git  && \  
    git clone https://github.com/orbbec/ros_astra_camera.git && \
    git clone https://github.com/orbbec/ros_astra_launch.git && \
    git clone https://bitbucket.org/iocchi/apriltags_ros.git && \
    git clone https://github.com/ros-drivers/urg_c.git && \
    git clone https://github.com/ros-drivers/urg_node.git && \
    git clone https://github.com/Slamtec/rplidar_ros.git && \
    git clone https://github.com/ros-perception/laser_proc.git 


# MARRtino ROS node

RUN mkdir -p $HOME/src/srrg && cd $HOME/src/srrg && \
    git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git && \
    git clone https://gitlab.com/srrg-software/srrg2_orazio.git


# Set up .bashrc

RUN echo "export MARRTINO_APPS_HOME=$HOME/src/marrtino_apps" >> $HOME/.bashrc
RUN echo "export ROBOT_TYPE=stage" >> $HOME/.bashrc
RUN echo "if [ -d /usr/lib/nvidia-384/ ]; then" >> $HOME/.bashrc
RUN echo "  export PATH=\"/usr/lib/nvidia-384/bin:\${PATH}\"" >> $HOME/.bashrc
RUN echo "  export LD_LIBRARY_PATH=\"/usr/lib/nvidia-384:/usr/lib32/nvidia-384:\${LD_LIBRARY_PATH}\" " >> $HOME/.bashrc
RUN echo "fi" >> $HOME/.bashrc



# Set and compile ROS packages

RUN cd $HOME/ros/catkin_ws/src && \
    ln -s $HOME/src/ros/usb_cam . && \
    ln -s $HOME/src/ros/ros_astra_camera . && \
    ln -s $HOME/src/ros/ros_astra_launch . && \
    ln -s $HOME/src/ros/apriltags_ros . && \
    ln -s $HOME/src/ros/urg_c . && \
    ln -s $HOME/src/ros/urg_node . && \
    ln -s $HOME/src/ros/rplidar_ros . && \
    ln -s $HOME/src/ros/laser_proc .  && \
    ln -s $HOME/src/stage_environments . && \
    ln -s $HOME/src/gradient_based_navigation . && \
    ln -s $HOME/src/srrg/srrg_cmake_modules . && \
    ln -s $HOME/src/srrg/srrg2_orazio . && \
    ln -s $HOME/src/thin_drivers/thin_msgs . && \
    ln -s $HOME/src/thin_drivers/thin_state_publisher .

RUN /bin/bash -c "source $HOME/ros/catkin_ws/devel/setup.bash; cd $HOME/ros/catkin_ws; catkin_make"


# Set working dir and container command

WORKDIR /home/robot

CMD /usr/bin/tmux


