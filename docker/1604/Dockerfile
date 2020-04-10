# Docker file for RoboCup@HomeEducation
# ROS Kinetic, navigation, perception & additional packages
# Version 0.9

FROM ros:kinetic-ros-base-xenial

ARG DEBIAN_FRONTEND=noninteractive

###### User root ######

# install libraries and ros packages 

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update

RUN apt-get install -y tmux less sudo eom nano \
    wget iputils-ping net-tools openssh-client \
    python-pip alsa-base alsa-utils pulseaudio pulseaudio-utils \
    sox mplayer libttspico-utils libwebsockets-dev libsuitesparse-dev

RUN apt-get install -y \
        ros-kinetic-desktop \
        ros-kinetic-navigation \
        ros-kinetic-perception \
        ros-kinetic-stage-ros \
        ros-kinetic-gmapping \
        ros-kinetic-joy \
        ros-kinetic-joystick-drivers

RUN apt-get install -y \
        ros-kinetic-audio-common \
        ros-kinetic-libuvc \
        ros-kinetic-rgbd-launch \
        ros-kinetic-web-video-server

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

RUN mkdir -p $HOME/.config/pulse && \
    cd $HOME/.config/pulse && \
    ln -s /opt/config/pulse/cookie .

RUN mkdir -p $HOME/Downloads

# Init ROS workspace

RUN mkdir -p $HOME/ros/catkin_ws/src

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd $HOME/ros/catkin_ws/src; catkin_init_workspace; cd ..; catkin_make"

RUN echo "source \$HOME/ros/catkin_ws/devel/setup.bash" >> $HOME/.bashrc

RUN rosdep update

# Python packages

RUN pip install --user setuptools==44 scipy==1.2.1 numpy==1.16.6 \
                       tensorflow==1.13.1 keras==2.2.4 \
                       np_utils==0.5.12.1 mock==1.0.0 \
                       sox==1.3.7 pyalsaaudio==0.8.4

# Libraries

# g2o

RUN mkdir -p $HOME/lib && cd $HOME/lib && \
    wget http://www.diag.uniroma1.it/iocchi/marrtino/lib/g2o-marrtino-src.tgz && \
    tar xzf g2o-marrtino-src.tgz && \
    rm g2o-marrtino-src.tgz && \
    cd g2o && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make

RUN echo "" >> $HOME/.bashrc && \
    echo "export G2O_ROOT=\$HOME/lib/g2o" >> $HOME/.bashrc && \
    echo "export G2O_BIN=\$HOME/lib/g2o/bin" >> $HOME/.bashrc && \
    echo "export G2O_LIB=\$HOME/lib/g2o/lib" >> $HOME/.bashrc && \
    echo "export LD_LIBRARY_PATH=\$HOME/lib/g2o/lib:\${LD_LIBRARY_PATH}" >> $HOME/.bashrc && \
    echo "" >> $HOME/.bashrc


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


# srrg_mapper

RUN mkdir -p $HOME/src/srrg && cd $HOME/src/srrg && \
    git clone https://gitlab.com/srrg-software/srrg_core.git && \
    cd srrg_core && git checkout a8f88898 && cd .. && \
    git clone https://gitlab.com/srrg-software/srrg_scan_matcher.git && \
    cd srrg_scan_matcher && git checkout 31e7c7ac && cd .. && \
    git clone https://gitlab.com/srrg-software/srrg_mapper2d.git && \
    cd srrg_mapper2d && git checkout 5ea162d1 && cd .. && \
    git clone https://gitlab.com/srrg-software/srrg_mapper2d_ros.git && \
    cd srrg_mapper2d_ros && git checkout 9aa14795 && cd .. 


# patches
RUN cd $HOME/src/srrg/srrg_mapper2d_ros && \
    rm CMakeLists.txt package.xml && \
    wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/CMakeLists.txt && \
    wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/package.xml && \
    cd $HOME/src/srrg/srrg_mapper2d_ros/src && \
    rm srrg_mapper2d_node.cpp message_handler.cpp && \
    wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/srrg_mapper2d_node.cpp && \
    wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/message_handler.cpp && \
    cd $HOME/src/srrg/srrg_scan_matcher/src && \
    rm laser_message_tracker.cpp && \
    wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_scan_matcher/laser_message_tracker.cpp 



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
    ln -s $HOME/src/thin_drivers/thin_state_publisher . && \
    ln -s $HOME/src/srrg/srrg_core . && \
    ln -s $HOME/src/srrg/srrg_scan_matcher . && \
    ln -s $HOME/src/srrg/srrg_mapper2d . && \
    ln -s $HOME/src/srrg/srrg_mapper2d_ros . 

RUN echo "export MARRTINO_VERSION=\"docker\"" >> $HOME/.bashrc

RUN /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make"


# Set working dir and container command

WORKDIR /home/robot

CMD /usr/bin/tmux


