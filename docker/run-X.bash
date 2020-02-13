#!/bin/bash

VERSION=0.4
PLAYGROUND_FOLDER=$HOME/playground

docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    --privileged \
    -e DISPLAY=$DISPLAY \
    --net=host \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    ros-kinetic-rchomeedu:$VERSION


