#!/bin/bash

# Use  ./run.bash [version]

IMAGENAME=ros-kinetic-rchomeedu
VERSION=0.4

if [ ! "$1" == "" ]; then
  VERSION=$1
fi

echo "Running image $IMAGENAME:$VERSION ..."

ROBOT_DEVICE=/dev/ttyACM0

if [ ! -f $ROBOT_DEVICE ]; then
  ROBOT_DEVICE=/dev/null
fi

if [ -d /usr/lib/nvidia-384 ]; then
  NVIDIASTR="-v /usr/lib/nvidia-384:/usr/lib/nvidia-384 \
           -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 \
           --device /dev/dri"
  echo "Nvidia libraries added"
else
  NVIDIASTR=""
fi

PLAYGROUND_FOLDER=$HOME/playground

USER_UID=$(id -u)

docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    $NVIDIASTR \
    -e DISPLAY=$DISPLAY \
    --privileged \
    --net=host \
    --device=$ROBOT_DEVICE \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    $IMAGENAME:$VERSION


