# Quadruped_Control_Soft_Contacts

Whole-Body Controller with soft contacts constraints implemented in C++.

## Table of Contents
- [Dependencies](#dependencies)
- [Installation](#installation)

## Dependencies

- `ROS2`, `gazebo_ros_pkgs`, `ros2-control`, `ros2-controllers`
- `Pinocchio`
- `Eigen`

## Installation
```shell
git clone --recursive https://github.com/ddebenedittis/Quadruped_Control_Soft_Contacts Quadruped_Control_Soft_Contacts/src
cd Quadruped_Control_Soft_Contacts
colcon build --symlink-install
```