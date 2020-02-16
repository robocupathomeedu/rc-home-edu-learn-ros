#!/bin/bash

# Use  ./run.bash [version]

IMAGENAME=ros-kinetic-rchomeedu
VERSION=0.4

if [ ! "$1" == "" ]; then
  VERSION=$1
fi

echo "Running image $IMAGENAME:$VERSION ..."

ROBOT_DEVICE=/dev/ttyACM0

if [ -f $ROBOT_DEVICE ]; then
  echo "Robot device $ROBOT_DEVICE enabled"
else
  ROBOT_DEVICE=/dev/null
fi

if [ -d /usr/lib/nvidia-384 ]; then
  NVIDIASTR="-v /usr/lib/nvidia-384:/usr/lib/nvidia-384 \
           -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 \
           --device /dev/dri"
  echo "Nvidia support enabled"
else
  NVIDIASTR=""
fi

if [ -d /run/user/$(id -u)/pulse ]; then
  AUDIOSTR="--device=/dev/snd \
           -v /run/user/$(id -u)/pulse:/run/user/1000/pulse \
           -v $HOME/.config/pulse/cookie:/home/robot/.config/pulse/cookie"
  echo "Audio support enabled"
else
  AUDIOSTR=""
fi

chmod go+rw ~/.config/pulse/cookie # this file needed by docker user

PLAYGROUND_FOLDER=$HOME/playground

docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    $NVIDIASTR \
    -e DISPLAY=$DISPLAY \
    --privileged \
    --net=host \
    --device=$ROBOT_DEVICE \
    $AUDIOSTR \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    $IMAGENAME:$VERSION


