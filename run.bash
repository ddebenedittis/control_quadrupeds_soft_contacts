# To use GUI
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

xhost +
docker run \
    # Share the host’s network stack and interfaces. Allows multiple containers to interact with each other.
    --net=host \
    # Interactive processes (like a shell).
    -it \
    # Clean up the container after exit.
    --rm \
    # Use GUI and NVIDIA
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --runtime=nvidia \
    --privileged \
    # Mount the folders in this directory
    -v ${PWD}:${PWD} \
    # Preserve bash history (for autocomplete).
    --env="HISTFILE=/home/.bash_history" \
    --env="HISTFILESIZE=$HISTFILESIZE" \
    -v ~/.bash_history:/home/.bash_history \
    # Audio support in Docker
    --device /dev/snd \
    -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
    -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native \
    --group-add $(getent group audio | cut -d: -f3) \
    #
    qcsc \
    bash