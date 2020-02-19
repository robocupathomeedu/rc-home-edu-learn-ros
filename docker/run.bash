#!/bin/bash

# Use  ./run.bash [version]

IMAGENAME=ros-kinetic-rchomeedu
VERSION=0.4
if [ ! "$1" == "" ]; then
  VERSION=$1
fi

# change setings here if needed
ROBOT_DEVICE=/dev/ttyACM0
CAMERA_DEVICE=/dev/video0
PLAYGROUND_FOLDER=$HOME/playground


echo "Running image $IMAGENAME:$VERSION ..."

if [ -f $ROBOT_DEVICE ]; then
  echo "Robot device $ROBOT_DEVICE enabled"
  ROBOT_DEVICE_STR="--device=$ROBOT_DEVICE"
fi

if [ -f $CAMERA_DEVICE ]; then
  echo "Camera device $CAMERA_DEVICE enabled"
  CAMERA_DEVICE_STR="--device=$CAMERA_DEVICE"
fi

if [ -d /usr/lib/nvidia-384 ]; then
  NVIDIA_STR="-v /usr/lib/nvidia-384:/usr/lib/nvidia-384 \
           -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 \
           --device /dev/dri"
  echo "Nvidia support enabled"
fi

if [ -d /run/user/$(id -u)/pulse ]; then
  AUDIO_STR="--device=/dev/snd \
           -v /run/user/$(id -u)/pulse:/run/user/1000/pulse \
           -v $HOME/.config/pulse/cookie:/home/robot/.config/pulse/cookie"
  echo "Audio support enabled"
fi

chmod go+rw ~/.config/pulse/cookie # this file needed by docker user


docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    $NVIDIA_STR \
    -e DISPLAY=$DISPLAY \
    --privileged \
    --net=host \
    $ROBOT_DEVICE_STR \
    $CAMERA_DEVICE_STR \
    $AUDIO_STR \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    $IMAGENAME:$VERSION


