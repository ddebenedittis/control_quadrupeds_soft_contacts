FROM osrf/ros:humble-desktop

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Prevents bash to ask for user input which may break the building process
ENV DEBIAN_FRONTEND=noninteractive

# Install sudo, some packages for Pinocchio, pip, and some ROS packages
RUN apt-get update && apt-get install -qqy \
    wget \
    git \
    bash-completion \
    build-essential \
    sudo \
    lsb-release \
    gnupg2 \
    curl \
    python3-pip \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Install Pinocchio
RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
RUN apt-get update && apt-get install -qqy \
    robotpkg-py310-pinocchio \
    && rm -rf /var/lib/apt/lists/*

# Set env variables for Pinocchio
ENV PATH="/opt/openrobots/bin:$PATH"
ENV PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH"
ENV LD_LIBRARY_PATH="/opt/openrobots/lib:$LD_LIBRARY_PATH"
ENV PYTHONPATH="/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH"
ENV CMAKE_PREFIX_PATH="/opt/openrobots:$CMAKE_PREFIX_PATH"

# Install quadprog
RUN pip3 install \
    numpy \
    scipy \
    numpy-quaternion \
    quadprog \
    --upgrade

# Create a new user
ARG UID=1000
ARG GID=1000
ARG USER=ros
ARG PWDR=/
RUN addgroup --gid ${GID} ${USER}
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ${USER}
RUN usermod -a -G dialout ${USER}
RUN echo ${USER}" ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Choose to run as user
ENV USER ${USER}
USER ${USER}

# Change HOME environment variable
ENV HOME /home/${USER}

# Set up environment
COPY config/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ${USER} /sbin/update_bashrc
RUN echo 'echo "source '${PWDR}'/install/setup.bash" >> ~/.bashrc' >> /sbin/update_bashrc
RUN cat /sbin/update_bashrc
RUN sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc


# Change entrypoint to source ~/.bashrc and start in ~
COPY config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ${USER} /ros_entrypoint.sh ;
RUN echo "cd "${PWDR} >> /ros_entrypoint.sh
RUN echo 'exec bash -i -c $@' >> /ros_entrypoint.sh

RUN cat /ros_entrypoint.sh

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]