#!/bin/bash

# Use  ./build.bash [Dockerfile] [version]

IMAGENAME=ros-kinetic-rchomeedu

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=0.9
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

docker build -t $IMAGENAME:$VERSION -f $DOCKERFILE .

