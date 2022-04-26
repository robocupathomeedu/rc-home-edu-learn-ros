#!/bin/bash

# Use  ./build.bash [Dockerfile] [version]

IMAGENAME=rchomeedu_vision

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=latest
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

docker build --build-arg UID=`id -u` --build-arg GID=`id -g` -t $IMAGENAME:$VERSION -f $DOCKERFILE .
docker tag $IMAGENAME:$VERSION iocchi/stagepersondetection

