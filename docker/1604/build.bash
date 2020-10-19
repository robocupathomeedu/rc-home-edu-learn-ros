#!/bin/bash

# Use  ./build.bash [version]

IMAGENAME=rchomeedu-1604-kinetic

VERSION=`cat version.txt`
if [ ! "$1" == "" ]; then
  VERSION=$1
fi

DOCKERFILE=Dockerfile.$VERSION

#if [ `docker image ls | grep $IMAGENAME | grep "base" | wc -l` == "0" ]; then
#  echo "Building image $IMAGENAME:base"
#  docker build --network=host -t $IMAGENAME:base -f Dockerfile.base . 
#fi


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


