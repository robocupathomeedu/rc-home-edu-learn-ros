#!/bin/bash

# Use  ./create.bash [version]

IMAGEBASE=rchomeedu-1804-melodic

IMAGENAME=iocchi/$IMAGEBASE
VERSION=latest

CONTAINERNAME="${IMAGEBASE}"

if [ "$1" != "" ]; then
  VERSION=$1
fi

echo "Image name: $IMAGENAME:$VERSION"
echo "Container name: $CONTAINERNAME"

echo "Creating container $CONTAINERNAME from image $IMAGENAME:$VERSION ..."


# change setings here if needed
ROBOT_DEVICE=/dev/ttyACM0
LASER_DEVICE=/dev/ttyUSB0
CAMERA_DEVICE=/dev/video0
JOYSTICK_DEVICE=/dev/input/js0
PLAYGROUND_FOLDER=$HOME/playground


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

docker container rm $CONTAINERNAME 

docker create -it \
    --name $CONTAINERNAME \
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
    -v $HOME/src/rc-home-edu-learn-ros:/home/robot/src/rc-home-edu-learn-ros \
    -v $HOME/src/marrtino_apps:/home/robot/src/marrtino_apps \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    $IMAGENAME:$VERSION


echo "Container $IMAGENAME created."
echo "Use docker start/stop $CONTAINERNAME to start stop the container."
echo "Use docker exec -it $CONTAINERNAME <cmd> to execute a command."

