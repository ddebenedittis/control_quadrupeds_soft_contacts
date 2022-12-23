#!/bin/bash
REBUILD=0
while getopts 'r' opt; do
  case $opt in
    r) REBUILD=1 ;;
    *) echo 'Error in command line parsing' >&2
      exit 1
    esac
done
shift "$(( OPTIND - 1 ))" 

mkdir -p build install log .vscode

docker pull osrf/ros:humble-desktop

MYUID="$(id -u $USER)"
MYGID="$(id -g $USER)"

if [ "$REBUILD" -eq 1 ]; then
  docker build \
  --no-cache \
  --build-arg BASE_IMAGE=osrf/ros \
  --build-arg BASE_TAG=humble-desktop \
  --build-arg MYUID=${UID} \
  --build-arg MYGID=${GID} \
  --build-arg USER=${USER} \
  --build-arg "PWDR=$PWD" \
  -t qcsc .
else
  docker build \
  --build-arg BASE_IMAGE=osrf/ros \
  --build-arg BASE_TAG=humble-desktop \
  --build-arg MYUID=${UID} \
  --build-arg MYGID=${GID} \
  --build-arg USER=${USER} \
  --build-arg "PWDR=$PWD" \
  -t qcsc .
fi
