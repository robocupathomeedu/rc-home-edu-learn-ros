#!/bin/bash

# Use  ./build.bash [Dockerfile] [version] [--no-cache]

IMAGENAME=rchomeedu-1604-kinetic

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=1.0
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

docker build -t $IMAGENAME:base -f Dockerfile.base . && \
docker build -t $IMAGENAME:$VERSION -f $DOCKERFILE .
 
