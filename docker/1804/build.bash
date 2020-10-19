#!/bin/bash

# Use  ./build.bash [version]

IMAGENAME=rchomeedu-1804-melodic

VERSION=`cat version.txt`
if [ ! "$1" == "" ]; then
  VERSION=$1
fi

DOCKERFILE=Dockerfile.$VERSION

echo "=====================================" &&
echo "Building image $IMAGENAME:base" &&
echo "=====================================" &&
docker build --network=host -t $IMAGENAME:base --build-arg MACHTYPE=`uname -m` \
    -f Dockerfile.base .  &&
echo "=====================================" &&
echo "Building image $IMAGENAME:$VERSION" &&
echo "=====================================" &&
docker build --network=host -t $IMAGENAME:$VERSION --build-arg MACHTYPE=`uname -m` \
    -f $DOCKERFILE . 

docker tag $IMAGENAME:$VERSION $IMAGENAME:latest

