#!/bin/bash

VERSION=0.3

docker run -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /usr/lib/nvidia-384:/usr/lib/nvidia-384 \
    -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 \
    --device /dev/dri \
    --net=host \
    -v $PLAYGROUND_FOLDER:/home/robot/playground \
    ros-kinetic-rchomeedu:$VERSION

# add in the container
#export PATH="/usr/lib/nvidia-384/bin":${PATH}
#export LD_LIBRARY_PATH="/usr/lib/nvidia-384:/usr/lib32/nvidia-384":${LD_LIBRARY_PATH}


