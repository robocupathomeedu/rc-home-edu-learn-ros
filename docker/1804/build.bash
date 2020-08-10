#!/bin/bash

# Use  ./build.bash [version]

IMAGENAME=rchomeedu-1804-melodic

VERSION=1.0.1
if [ ! "$1" == "" ]; then
  VERSION=$1
fi

DOCKERFILE=Dockerfile.$VERSION

if [ `docker image ls | grep $IMAGENAME | grep "base" | wc -l` == "0" ]; then
  echo "Building image $IMAGENAME:base"
  docker build --network=host -t $IMAGENAME:base -f Dockerfile.base . 
fi

echo "Building image $IMAGENAME:$VERSION"

docker build --network=host -t $IMAGENAME:$VERSION -f $DOCKERFILE .

