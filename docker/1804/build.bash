#!/bin/bash

# Use  ./build.bash [version]

IMAGENAME=iocchi/rchomeedu-1804-melodic

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
echo "Building image $IMAGENAME:4" &&
echo "=====================================" &&
docker build --network=host -t $IMAGENAME:4 --build-arg MACHTYPE=`uname -m` \
    -f Dockerfile.4 .  &&
echo "=====================================" &&
echo "Building image $IMAGENAME:$VERSION" &&
echo "=====================================" &&
docker build --network=host -t $IMAGENAME:$VERSION --build-arg MACHTYPE=`uname -m` \
    -f $DOCKERFILE . 

docker tag $IMAGENAME:$VERSION $IMAGENAME:latest

# docker login
# docker push iocchi/rchomeedu-1804-melodic:latest

