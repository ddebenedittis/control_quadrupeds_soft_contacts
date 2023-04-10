#!/bin/bash

# ================================= Edit Here ================================ #

# Change these values to use different versions of ROS or different base images. The rest of the script should be left unchanged.
BASE_IMAGE=osrf/ros
BASE_TAG=humble-desktop
IMAGE_NAME=qcsc


# =============================== Help Function ============================== #

helpFunction()
{
   echo ""
   echo "Usage: $0 [-a] [-d] [-h] [-p] [-r] [-t]"
   echo -e "\t-a   --all           Install all the optional packages."
   echo -e "\t-d   --development   Install additional packages for development purposes."
   echo -e "\t-h   --help          Print the help."
   echo -e "\t-p   --plot          Install latex in the image. Bigger image, but plot.py can be used to plot some figures."
   echo -e "\t-r   --rebuild       Rebuild the image."
   echo -e "\t-t   --terrain-gen   Install bly (blender Python API) to generate terrains from heightmaps."
   exit 1 # Exit script after printing help
}


# =============================== Build Options ============================== #

# Initialie the build options
DEVELOPMENT=0
PLOT=0
REBUILD=0
TERRAIN_GEN=0

# Auxiliary functions
die() { echo "$*" >&2; exit 2; }  # complain to STDERR and exit with error
needs_arg() { if [ -z "$OPTARG" ]; then die "No arg for --$OPT option"; fi; }
no_arg() { if [ -n "$OPTARG" ]; then die "No arg allowed for --$OPT option"; fi; }

# Get the script options. This accepts both single dash (e.g. -a) and double dash options (e.g. --all)
while getopts adhprt-: OPT; do
  # support long options: https://stackoverflow.com/a/28466267/519360
  if [ "$OPT" = "-" ]; then   # long option: reformulate OPT and OPTARG
    OPT="${OPTARG%%=*}"       # extract long option name
    OPTARG="${OPTARG#$OPT}"   # extract long option argument (may be empty)
    OPTARG="${OPTARG#=}"      # if long option argument, remove assigning `=`
  fi
  case "$OPT" in
    a | all )           no_arg; DEVELOPMENT=1; PLOT=1; TERRAIN_GEN=1; ;;
    d | development )   no_arg; DEVELOPMENT=1 ;;
    h | help )          no_arg; helpFunction ;;
    p | plot )          no_arg; PLOT=1 ;;
    r | rebuild )       no_arg; REBUILD=1 ;;
    t | terrain_gen )   no_arg; TERRAIN_GEN=1 ;;
    ??* )               die "Illegal option --$OPT" ;;  # bad long option
    ? )                 exit 2 ;;  # bad short option (error reported via getopts)
  esac
done
shift $((OPTIND-1)) # remove parsed options and args from $@ list


# ========================= Pull And Build The Image ========================= #

docker pull $BASE_IMAGE:$BASE_TAG

MYUID="$(id -u $USER)"
MYGID="$(id -g $USER)"

if [ "$REBUILD" -eq 1 ]; then
    cache="--no-cache"
else
    cache=""
fi

docker build \
${cache} \
--build-arg BASE_IMAGE=$BASE_IMAGE \
--build-arg BASE_TAG=$BASE_TAG \
--build-arg MYUID=${UID} \
--build-arg MYGID=${GID} \
--build-arg USER=${USER} \
--build-arg "PWDR=$PWD" \
--build-arg DEVELOPMENT=$DEVELOPMENT \
--build-arg PLOT=$PLOT \
--build-arg TERRAIN_GEN=$TERRAIN_GEN \
-t $IMAGE_NAME .
