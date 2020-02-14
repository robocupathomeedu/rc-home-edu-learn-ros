#!/bin/bash

VERSION=0.4
ROBOT_DEVICE=/dev/ttyACM0
PLAYGROUND_FOLDER=$HOME/playground

docker run -it \
    --net=host \
    --device=$ROBOT_DEVICE \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    ros-kinetic-rchomeedu:$VERSION

