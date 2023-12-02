# Create /tmp/.docker.xauth if it does not already exist.
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

IMAGE_NAME=qcsc

xhost +
docker run \
    `# Share the host’s network stack and interfaces. Allows multiple containers to interact with each other.` \
    --net=host \
    `# Interactive processes, like a shell.` \
    -it \
    `# Clean up the container after exit.` \
    --rm \
    `# Use GUI and NVIDIA.` \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="/dev/dri:/dev/dri" \
    --gpus all \
    `# Mount the folders in this directory.` \
    -v ${PWD}:${PWD} \
    `# Preserve bash history for autocomplete).` \
    --env="HISTFILE=/home/.bash_history" \
    --env="HISTFILESIZE=$HISTFILESIZE" \
    -v ~/.bash_history:/home/.bash_history \
    `# Audio support in Docker.` \
    --device /dev/snd \
    -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
    -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native \
    --group-add $(getent group audio | cut -d: -f3) \
    `# Enable SharedMemory between host and container.` \
    `# https://answers.ros.org/question/370595/ros2-foxy-nodes-cant-communicate-through-docker-container-border/` \
    -v /dev/shm:/dev/shm \
    `# Logging.` \
    -v /home/$USER/.ros:/home/$USER/.ros \
    `# Mount folders useful for VS Code.` \
    -v /home/$USER/.vscode:/home/$USER/.vscode \
    -v /home/$USER/.vscode-server:/home/$USER/.vscode-server \
    -v /home/$USER/.config/Code:/home/$USER/.config/Code \
    `# Enable the kernel's address space randomization for gdb.` \
    --cap-add=SYS_PTRACE \
    --security-opt seccomp=unconfined \
    `# Add SSH Key` \
    -v $SSH_AUTH_SOCK:/ssh-agent \
    -e SSH_AUTH_SOCK=/ssh-agent \
    ${IMAGE_NAME} \
    bash
