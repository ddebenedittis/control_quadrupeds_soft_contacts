ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=humble-desktop

FROM ${BASE_IMAGE}:${BASE_TAG}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Prevents bash to ask for user input which may break the building process
ENV DEBIAN_FRONTEND=noninteractive

# Install sudo, some packages for Pinocchio, pip, and some ROS packages
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    apt-get update && apt-get install --no-install-recommends -qqy \
    bash-completion \
    python3-pip \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-pinocchio \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    sudo \
    xterm

# Install the python packages.
RUN pip3 install \
    numpy \
    numpy-quaternion \
    quadprog \
    scipy \
    --upgrade

# If $DEVELOPMENT is 1, install additional development packages.
ARG DEVELOPMENT=0
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    if [ "${DEVELOPMENT}" = "1" ] ; then \
        apt-get update && apt-get install --no-install-recommends -qqy \
        ros-humble-ament-clang-tidy ; \
    fi

# If $PLOT is 1, install the packages required to use plot.py in the container.
ARG PLOT=0
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    if [ "${PLOT}" = "1" ] ; then \
        apt-get update && apt-get install --no-install-recommends -qqy \
        dvipng \
        texlive-latex-extra \
        texlive-fonts-recommended \
        cm-super ; \
    fi

# If #TERRAIN_GEN is 1, install bpy for generating the terrain meshes with Blender.
ARG TERRAIN_GEN=0
RUN if [ "${TERRAIN_GEN}" = "1" ] ; then \
        pip3 install \
        bpy \
        --upgrade ; \
    fi

# Create the same user as the host itself. (By default Docker creates the container as root, which is not recommended.)
ARG UID=1000
ARG GID=1000
ARG USER=ros
ARG PWDR=/
RUN addgroup --gid ${GID} ${USER} \
 && adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ${USER} \
 && usermod -a -G dialout ${USER} \
 && echo ${USER}" ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/99_aptget \
 && chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Choose to run as user
ENV USER ${USER}
USER ${USER}

# Change HOME environment variable
ENV HOME /home/${USER}

# Set up environment
COPY config/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ${USER} /sbin/update_bashrc \
 && echo 'echo "source '${PWDR}'/install/setup.bash" >> ~/.bashrc' >> /sbin/update_bashrc \
 && cat /sbin/update_bashrc \
 && sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc

# Change entrypoint to source ~/.bashrc and start in ~
COPY config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ${USER} /ros_entrypoint.sh \
 && echo "cd "${PWDR} >> /ros_entrypoint.sh \
 && echo 'exec bash -i -c $@' >> /ros_entrypoint.sh \
 && cat /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
