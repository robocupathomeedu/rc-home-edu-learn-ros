#!/bin/bash

VERSION=0.4
ROBOT_DEVICE=/dev/ttyACM0

if [ -f $ROBOT_DEVICE ]; then
echo ""
else
ROBOT_DEVICE=/dev/null
fi

PLAYGROUND_FOLDER=$HOME/playground

docker run -it \
    --net=host \
    --device=$ROBOT_DEVICE \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    ros-kinetic-rchomeedu:$VERSION

