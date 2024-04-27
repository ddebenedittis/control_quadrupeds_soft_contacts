ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=iron-desktop

FROM ${BASE_IMAGE}:${BASE_TAG}

# ================================== Nvidia ================================== #

# nvidia-container-runtime
ENV NVARCH x86_64

ENV NVIDIA_REQUIRE_CUDA "cuda>=12.1 brand=tesla,driver>=450,driver<451 brand=tesla,driver>=470,driver<471 brand=unknown,driver>=470,driver<471 brand=nvidia,driver>=470,driver<471 brand=nvidiartx,driver>=470,driver<471 brand=geforce,driver>=470,driver<471 brand=geforcertx,driver>=470,driver<471 brand=quadro,driver>=470,driver<471 brand=quadrortx,driver>=470,driver<471 brand=titan,driver>=470,driver<471 brand=titanrtx,driver>=470,driver<471 brand=tesla,driver>=510,driver<511 brand=unknown,driver>=510,driver<511 brand=nvidia,driver>=510,driver<511 brand=nvidiartx,driver>=510,driver<511 brand=geforce,driver>=510,driver<511 brand=geforcertx,driver>=510,driver<511 brand=quadro,driver>=510,driver<511 brand=quadrortx,driver>=510,driver<511 brand=titan,driver>=510,driver<511 brand=titanrtx,driver>=510,driver<511 brand=tesla,driver>=515,driver<516 brand=unknown,driver>=515,driver<516 brand=nvidia,driver>=515,driver<516 brand=nvidiartx,driver>=515,driver<516 brand=geforce,driver>=515,driver<516 brand=geforcertx,driver>=515,driver<516 brand=quadro,driver>=515,driver<516 brand=quadrortx,driver>=515,driver<516 brand=titan,driver>=515,driver<516 brand=titanrtx,driver>=515,driver<516 brand=tesla,driver>=525,driver<526 brand=unknown,driver>=525,driver<526 brand=nvidia,driver>=525,driver<526 brand=nvidiartx,driver>=525,driver<526 brand=geforce,driver>=525,driver<526 brand=geforcertx,driver>=525,driver<526 brand=quadro,driver>=525,driver<526 brand=quadrortx,driver>=525,driver<526 brand=titan,driver>=525,driver<526 brand=titanrtx,driver>=525,driver<526"
ENV NV_CUDA_CUDART_VERSION 12.1.105-1
ENV NV_CUDA_COMPAT_PACKAGE cuda-compat-12-1

LABEL maintainer "NVIDIA CORPORATION <cudatools@nvidia.com>"

RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/${NVARCH}/3bf863cc.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/${NVARCH} /" > /etc/apt/sources.list.d/cuda.list && \
    apt-get purge --autoremove -y curl \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 12.1.1

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-12-1=${NV_CUDA_CUDART_VERSION} \
    ${NV_CUDA_COMPAT_PACKAGE} \
    && rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf \
    && echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,graphics,utility

# ============================================================================ #

# Prevents bash to ask for user input which may break the building process
ENV DEBIAN_FRONTEND=noninteractive

# Install sudo, some packages for Pinocchio, pip, and some ROS packages
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    apt-get update && apt-get install --no-install-recommends -qqy \
    bash-completion \
    python3-pip \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-pinocchio \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-plotjuggler-ros \
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
        gdb \
        ros-iron-ament-clang-tidy \
        && pip3 install matplotlib --upgrade ; \
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

# If TRACING is 1, install ros2trace and tracetools-analysis.
ARG TRACING=0
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    if [ "${TRACING}" = "1" ] ; then \
        apt-get update && apt-get install --no-install-recommends -qqy \
        babeltrace \
        ros-${ROS_DISTRO}-ros2trace \
        ros-${ROS_DISTRO}-tracetools \
        ros-${ROS_DISTRO}-tracetools-launch \
        ros-${ROS_DISTRO}-tracetools-analysis \
        && pip3 install \
        bokeh \
        pandas \
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

# Install the required dependencies with rosdep.
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    --mount=type=bind,source=./src,target=/home/${USER}/catkin_ws/src,rw \
    cd /home/${USER}/catkin_ws \
 && sudo apt-get update \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y

# Set up environment
COPY .config/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ${USER} /sbin/update_bashrc \
 && echo 'echo "source '${PWDR}'/install/setup.bash" >> ~/.bashrc' >> /sbin/update_bashrc \
 && cat /sbin/update_bashrc \
 && sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc

# Change entrypoint to source ~/.bashrc and start in ~
COPY .config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ${USER} /ros_entrypoint.sh \
 && echo "cd "${PWDR} >> /ros_entrypoint.sh \
 && echo 'exec bash -i -c $@' >> /ros_entrypoint.sh \
 && cat /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
