#!/bin/bash

# Use  ./build.bash [version] [Dockerfile]

IMAGENAME=ros-kinetic-rchomeedu

VERSION=0.9
if [ ! "$1" == "" ]; then
  VERSION=$1
fi

DOCKERFILE=Dockerfile
if [ ! "$2" == "" ]; then
  DOCKERFILE=$2
fi


docker build -t $IMAGENAME:$VERSION -f $DOCKERFILE .

