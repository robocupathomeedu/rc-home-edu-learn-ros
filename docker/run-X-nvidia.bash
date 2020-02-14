#!/bin/bash

VERSION=0.4
ROBOT_DEVICE=/dev/ttyACM0
PLAYGROUND_FOLDER=$HOME/playground

docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /usr/lib/nvidia-384:/usr/lib/nvidia-384 \
    -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 \
    --device /dev/dri \
    --net=host \
    --device=$ROBOT_DEVICE \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    ros-kinetic-rchomeedu:$VERSION


