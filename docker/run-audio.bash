#!/bin/bash

VERSION=0.7
ROBOT_DEVICE=/dev/ttyACM0

if [ -f $ROBOT_DEVICE ]; then
echo ""
else
ROBOT_DEVICE=/dev/null
fi


PLAYGROUND_FOLDER=$HOME/playground
USER_UID=$(id -u)


docker run -it \
    --net=host \
    --privileged \
    --device=$ROBOT_DEVICE \
    --device=/dev/snd \
    -v /run/user/${USER_UID}/pulse:/run/user/1000/pulse \
    -v ~/.config/pulse/cookie:/home/robot/.config/pulse/cookie \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    ros-kinetic-rchomeedu:$VERSION


