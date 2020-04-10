#!/bin/bash

# Use  ./build.bash [Dockerfile] [version] [--no-cache]

IMAGENAME=ros-kinetic-rchomeedu

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=0.9
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

docker build $3 -t $IMAGENAME:$VERSION -f $DOCKERFILE .

