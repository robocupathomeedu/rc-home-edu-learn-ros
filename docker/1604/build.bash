#!/bin/bash

# Use  ./build.bash [Dockerfile] [version]

IMAGENAME=rchomeedu-1604-kinetic

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=1.0.1
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

if [ `docker image ls | grep $IMAGENAME | grep "base" | wc -l` == "0" ]; then
  echo "Building image $IMAGENAME:base"
  docker build --network=host -t $IMAGENAME:base -f Dockerfile.base . 
fi

echo "Building image $IMAGENAME:$VERSION"

docker build --network=host -t $IMAGENAME:$VERSION -f $DOCKERFILE .

