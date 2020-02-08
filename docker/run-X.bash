#!/bin/bash

VERSION=0.3

docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    --privileged \
    -e DISPLAY=$DISPLAY \
    --net=host \
    ros-kinetic-rchomeedu:$VERSION



