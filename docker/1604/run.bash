#!/bin/bash

# Use  ./run.bash [-local] [version]

IMAGEBASE=rchomeedu-1604-kinetic

IMAGENAME=iocchi/$IMAGEBASE
VERSION=1.0

if [ "$1" == "-local" ]; then
  IMAGENAME=$IMAGEBASE
  VER=$2
else
  VER=$1
fi

if [ "$VER" != "" ]; then
  VERSION=$VER
fi

echo "$IMAGENAME:$VERSION"


# change setings here if needed
ROBOT_DEVICE=/dev/ttyACM0
LASER_DEVICE=/dev/ttyUSB0
CAMERA_DEVICE=/dev/video0
JOYSTICK_DEVICE=/dev/input/js0
PLAYGROUND_FOLDER=$HOME/playground


echo "Running image $IMAGENAME:$VERSION ..."

if [ -e ${ROBOT_DEVICE} ]; then
  echo "Robot device ${ROBOT_DEVICE} found"
fi

if [ -e ${LASER_DEVICE} ]; then
  echo "Laser device ${LASER_DEVICE} found"
fi

if [ -e ${CAMERA_DEVICE} ]; then
  echo "Camera device ${CAMERA_DEVICE} found"
fi

if [ -e ${JOYSTICK_DEVICE} ]; then
  echo "Joystick device ${JOYSTICK_DEVICE} found"
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
           -v $HOME/.config/pulse/cookie:/opt/config/pulse/cookie"
#           -v $HOME/.config/pulse/cookie:/home/robot/.config/pulse/cookie"
  echo "Audio support enabled"
fi

chmod go+rw ~/.config/pulse/cookie # this file needed by docker user
chmod go+xrw /run/user/$(id -u)/pulse # this file needed by docker user


docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    $NVIDIA_STR \
    -e DISPLAY=$DISPLAY \
    --privileged \
    --net=host \
    -v /dev:/dev \
    $AUDIO_STR \
    -e ROBOT_DEVICE=$ROBOT_DEVICE \
    -e LASER_DEVICE=$LASER_DEVICE \
    -e CAMERA_DEVICE=$CAMERA_DEVICE \
    -e JOYSTICK_DEVICE=$JOYSTICK_DEVICE \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    $IMAGENAME:$VERSION




