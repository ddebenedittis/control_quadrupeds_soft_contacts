# Quadruped_Control_Soft_Contacts

Whole-Body Controller with soft contacts constraints implemented in C++.

## Table of Contents
- [Installation with Docker](#installation-with-docker)
- [Dependencies](#dependencies)
- [Installation](#installation)

## Installation with Docker
Clone the repo:
```shell
git clone --recursive https://github.com/ddebenedittis/quadruped_control_soft_contacts
```
Build the docker image (have docker installed and able to manage Docker as a non-root user (see post-installation steps for Linux)) (-r option to update the underlying images):
```shell
./build.sh
```
Run the container:
```shell
./run.sh
```
Build the ROS workspace:
```shell
colcon build --symlink-install && source devel/setup.bash
```

## Dependencies

- `git`
- `Eigen`
- `ROS2 Galactic`, `ros2-control`, `ros2-controllers`, `gazebo-ros-pkgs`, `gazebo-ros2-control`, `xacro`
- `Pinocchio`
- `numpy`, `scipy`, `quadprog`

## Installation
```shell
git clone --recursive https://github.com/ddebenedittis/Quadruped_Control_Soft_Contacts Quadruped_Control_Soft_Contacts/src
cd Quadruped_Control_Soft_Contacts
colcon build --symlink-install
```