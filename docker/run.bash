#!/bin/bash

VERSION=0.4

docker run -it \
    --net=host \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    ros-kinetic-rchomeedu:$VERSION


