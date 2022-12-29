# Control Quadrupeds Soft Contacts

Control of quadrupedal robots with soft contact constraints.

## Table of Contents
- [Installation with Docker](#installation-with-docker)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Author](#author)

## Installation with Docker
Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine) with post-installation steps for Linux and [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (nvidia-docker2).

Clone the repo:
```shell
git clone --recursive https://github.com/ddebenedittis/control_quadrupeds_soft_contacts
```
Build the docker image (-r option to update the underlying images):
```shell
./build.sh
```
Run the container:
```shell
./run.bash
```
Build the ROS workspace:
```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash
```

## Dependencies

- `git`
- `Eigen`
- `ROS2 Galactic`, and the following ROS2 packages: `ros2-control`, `ros2-controllers`, `gazebo-ros-pkgs`, `gazebo-ros2-control`, `xacro`, `joint-state-publisher`, `joint-state-publisher-gui`
- [`Pinocchio`](https://github.com/stack-of-tasks/pinocchio)
- `numpy`, `scipy`, `numpy_quaternion`, [`quadprog`](https://github.com/quadprog/quadprog)

## Installation
```shell
git clone --recursive https://github.com/ddebenedittis/Quadruped_Control_Soft_Contacts Quadruped_Control_Soft_Contacts/src
cd Quadruped_Control_Soft_Contacts
colcon build --symlink-install
```

## Usage
- Anymal C static walk simulation:
```shell
ros2 launch robot_gazebo anymal.launch.py
```
- SOLO12 static walk simulation:
```shell
ros2 launch robot_gazebo solo.launch.py
```

## Author
Davide De Benedittis