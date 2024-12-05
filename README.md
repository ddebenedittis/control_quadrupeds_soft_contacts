# Control Quadrupeds Soft Contacts

Control of quadrupedal robots with soft contact constraints.


## Table of Contents

- [Control Quadrupeds Soft Contacts](#control-quadrupeds-soft-contacts)
  - [Table of Contents](#table-of-contents)
  - [Installation with Docker](#installation-with-docker)
  - [Dependencies](#dependencies)
  - [Installation](#installation)
  - [Usage](#usage)
    - [Simulations](#simulations)
    - [Plot](#plot)
    - [Add a new robot model](#add-a-new-robot-model)
  - [Troubleshooting](#troubleshooting)
  - [Known Bugs](#known-bugs)
  - [Author](#author)
  - [Citation](#citation)


## Installation with Docker

Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine) and [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).

Install NVIDIA proprietary drivers if the NVIDIA graphics card should be used.

Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit) (nvidia-docker2) for NVIDIA support in the container. \
If you do not want to use NVIDIA, edit the Docker image to remove the NVIDIA section and the `run.bash` script, removing the `--gpus all` flag in the docker run command. In addition, remove the `additional_env` from the Gazebo process `gzserver` in `robot_launch/launch/robot.launch.py`.

If you do not have access to all the submodules, including the private ones, clone the repo with
```shell
git clone https://github.com/ddebenedittis/control_quadrupeds_soft_contacts
cd control_quadrupeds_soft_contacts
git submodule update --init --recursive --remote src/external/quadprog/ src/external/rapidyaml/ src/rviz_legged/ src/robot/robots/anymal_c_simple_description/ src/robot/robots/solo_description/
```

ONLY IF you have access to all the submodules, including the private ones, clone the repo with
```shell
git clone --recursive https://github.com/ddebenedittis/control_quadrupeds_soft_contacts
```

If not already done, navigate to the workspace with
```shell
cd Quadruped_Control_Soft_Contacts
```

Build the docker image (`-d` to install additional packages for development) (`-h` for printing the help) (`-p` to install all the dependencies to use plot) (`-r` to rebuild the underlying images) (`-t` for installing bly to generate the terrain meshes from some heightmaps) (`--t` for installing tracing tools) (`-a` to install all the optional dependencies):
```shell
./build.bash [-a] [-d] [-h] [-p] [-r] [-t] [--tracing]
```
Run the container:
```shell
./run.bash
```
Build the ROS workspace:
```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash
```


## Dependencies

- `git`
- `Eigen`
- `xterm`
- `ROS2 Humble`, and the following ROS2 packages: `ros2-control`, `ros2-controllers`, `gazebo-ros-pkgs`, `gazebo-ros2-control`, `xacro`, `joint-state-publisher`, `joint-state-publisher-gui`
- [`Pinocchio`](https://github.com/stack-of-tasks/pinocchio)
- `numpy`, `scipy`, `numpy_quaternion`, [`quadprog`](https://github.com/quadprog/quadprog)
- `latex` only for plotting some figures.


## Installation

If you do not have access to all the submodules, including the private ones, clone the repo with
```shell
git clone https://github.com/ddebenedittis/control_quadrupeds_soft_contacts
cd control_quadrupeds_soft_contacts
git submodule update --init --recursive --remote src/external/quadprog/ src/external/rapidyaml/ src/rviz_legged/ src/robot/robots/anymal_c_simple_description/ src/robot/robots/solo_description/
```

ONLY IF you have access to all the submodules, including the private ones, clone the repo with
```shell
git clone --recursive https://github.com/ddebenedittis/control_quadrupeds_soft_contacts
```

Build the workspace with
```shell
cd control_quadrupeds_soft_contacts
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash
```


## Usage

### Simulations

- ANYmal C simulation:
```shell
ros2 launch robot_gazebo anymal_c.launch.py [terrain:={rigid,soft,very_soft,multi_terrains,heightmap}] [gait:={teleop_base,static_walk,walking_trot,teleop_walking_trot}] [velocity_cmd:="(0.0, 0.0, 0.0)"] [use_rviz:={False,True}] [save_csv:={False,True}] [reset:={False,True}]
```

![](https://raw.githubusercontent.com/ddebenedittis/media/main/control_quadrupeds_soft_contacts/anymal_c-trot-grass.webp)

- SOLO12 static walk simulation:
```shell
ros2 launch robot_gazebo solo12.launch.py [terrain:={rigid,soft,very_soft,multi_terrains,heightmap}] [gait:={teleop_base,static_walk,walking_trot,teleop_walking_trot}] [velocity_cmd:="(0.0, 0.0, 0.0)"] [use_rviz:={False,True}] [save_csv:={False,True}] [reset:={False,True}]
```

![](https://raw.githubusercontent.com/ddebenedittis/media/main/control_quadrupeds_soft_contacts/solo12-walk-rigid.webp)

- ANYmal C with SoftFoot-Qs static walk simulation:
```shell
ros2 launch robot_gazebo anymal_c_softfoot_q.launch.py [terrain:={rigid,soft,very_soft,multi_terrains,heightmap}] [gait:={teleop_base,static_walk,walking_trot,teleop_walking_trot}] [velocity_cmd:="(0.0, 0.0, 0.0)"] [use_rviz:={False,True}] [save_csv:={False,True}] [reset:={False,True}]
```

Optional arguments:
- `terrain`: select the terrain type: a rigid terrain (the gray one) or a soft terrain (the grass-like one).
- `gait`: select the gait type. With `teleop_base` the base pose can be teleoperated without moving the feet, `static_walk` implements a static walk, and `walking_trot` implements a dynamic gait. The speed of the `walking_trot` gait is commanded with the following parameter: `velocity_cmd:="(0.0, 0.0, 0.0)"`.  With `teleop_walking_trot` the robot is teleoperated by giving it a velocity command.
- `use_rviz`: when `True`, RViz is used to display the contact forces of the robot (still in Alpha version).
- `save_csv`: when `True`, some data is logged and saved in several .csv files in the `log/csv/` folder. This data can be plotted with `plot.py` (in the `logger_gazebo` package).
- `reset`: must be used in another terminal when there is an already running simulation. The simulation will be restarted. The time is not reset to avoid problems with the controllers.

<img src="https://raw.githubusercontent.com/ddebenedittis/media/main/rviz_legged/rviz_legged_walk.webp" width="500">
<img src="https://raw.githubusercontent.com/ddebenedittis/media/main/rviz_legged/rviz_legged_trot.webp" width="500">

- `contact_constraint_type`: not working. Must be changed directly in the specific robot config .yaml file.

In some simulations (e.g. the simulation of ANYmal with SoftFoot-Q or the simulation with a trotting gait), due to the relatively high computational cost, it may be helpful to start or to run the simulation with a reduced real-time factor.

### Plot

- Plot and save the figures in `log/svg/` or `log/pdf/` (must be executed while in the workspace folder):
```shell
ros2 run logger_gazebo plot
```

To save the plots, it is necessary to have a Docker image with latex installed. The image must have been built with the `-l` option.


### Add a new robot model

Add a new robot model in the `all_robots.yaml` file located in `src/robot/robot_model/robots/`. Pay attention to the order of the feet' names (should be Left Front, Right Front, Left Hind, Right Hind).

Add a new robot description package. It is recommended to place it in `src/robot/robots/`. The package should have both a .urdf file and a .xacro file, which needs to be augmented with the necessary ros2 plugins etc (see, for example, `anymal_c_simple_description/urdf/anymal_gazebo.xacro`).

Generate a new launch file similar to the ones already present in `src/robot/robot_gazebo/`.

Create a new `effort_controller.yaml` file, similar to the ones already present in `robot_control/config`. Edit at least the `robot_name` field and the `joints` fields (according to the names of the joints of your robot).


## Troubleshooting

- If you do not have an NVIDIA graphics card, or you do not have the propietary drivers (you can check this by using the command `nvidia-smi`), you should remove the `additional_env` from the Gazebo process `gzserver` in `robot_launch/launch/robot.launch.py`.


## Known Bugs

- None

## Author

[Davide De Benedittis](https://3.bp.blogspot.com/-xvFfjYBPegM/VvFp02nHUjI/AAAAAAAAIoc/Mysj-ESrXPQFQI_yOJFQQz2kwZuIQiAKA/s1600/He-Man.png)

## Citation

If you find this project useful in your research, please consider citing my related work (available [here](https://doi.org/10.1109/TSMC.2024.3504342)):

```bibtex
@ARTICLE{debenedittis2024soft,
  author={De Benedittis, Davide and Angelini, Franco and Garabini, Manolo},
  journal={IEEE Transactions on Systems, Man, and Cybernetics: Systems}, 
  title={Soft Bilinear Inverted Pendulum: A Model to Enable Locomotion With Soft Contacts}, 
  year={2024},
  volume={},
  number={},
  pages={1-14},
  keywords={Legged locomotion;Quadrupedal robots;Foot;Vectors;Optimization;Computational modeling;Trajectory;Tracking;Planning;Jacobian matrices;Contacts;legged locomotion;optimal control;predictive control;quadratic programming},
  doi={10.1109/TSMC.2024.3504342}}
```