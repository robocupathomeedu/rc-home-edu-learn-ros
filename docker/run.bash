#!/bin/bash

VERSION=0.3

docker run -it \
    --net=host \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    ros-kinetic-rchomeedu:$VERSION


