#!/bin/bash

helpFunction()
{
   echo ""
   echo "Usage: $0 [-d] [-l] [-r]"
   echo -e "\t-d Install additional packages for development purposes"
   echo -e "\t-l Install latex in the image. Bigger image, but plot.py can be used."
   echo -e "\t-r Rebuild the image"
   exit 1 # Exit script after printing help
}

# Build options
DEVELOPMENT=0
LATEX=0
REBUILD=0

while getopts 'd:l:r' opt
do
    case $opt in
        d) DEVELOPMENT=1 ;;
        l) LATEX=1 ;;
        r) REBUILD=1 ;;
        ?) helpFunction ;; # Print helpFunction in case parameter is non-existent
    esac
done
shift "$(( OPTIND - 1 ))" 

BASE_IMAGE=osrf/ros
BASE_TAG=humble-desktop
IMAGE_NAME=qcsc

docker pull $BASE_IMAGE:$BASE_TAG

MYUID="$(id -u $USER)"
MYGID="$(id -g $USER)"

if [ "$REBUILD" -eq 1 ]; then
    docker build \
    --no-cache \
    --build-arg BASE_IMAGE=$BASE_IMAGE \
    --build-arg BASE_TAG=$BASE_TAG \
    --build-arg MYUID=${UID} \
    --build-arg MYGID=${GID} \
    --build-arg USER=${USER} \
    --build-arg "PWDR=$PWD" \
    --build-arg DEVELOPMENT=$DEVELOPMENT \
    --build-arg LATEX=$LATEX \
    -t $IMAGE_NAME .
else
    docker build \
    --build-arg BASE_IMAGE=$BASE_IMAGE \
    --build-arg BASE_TAG=$BASE_TAG \
    --build-arg MYUID=${UID} \
    --build-arg MYGID=${GID} \
    --build-arg USER=${USER} \
    --build-arg "PWDR=$PWD" \
    --build-arg DEVELOPMENT=$DEVELOPMENT \
    --build-arg LATEX=$LATEX \
    -t $IMAGE_NAME .
fi
