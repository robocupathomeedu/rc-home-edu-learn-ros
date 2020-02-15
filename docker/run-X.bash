#!/bin/bash

VERSION=0.4

ROBOT_DEVICE=/dev/ttyACM0
if [ -f $ROBOT_DEVICE ]; then
ROBOT_DEVICE=/dev/ttyACM0
else
ROBOT_DEVICE=/dev/null
fi

PLAYGROUND_FOLDER=$HOME/playground

USER_UID=$(id -u)

docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    --privileged \
    -e DISPLAY=$DISPLAY \
    --net=host \
    --device=$ROBOT_DEVICE \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    ros-kinetic-rchomeedu:$VERSION


