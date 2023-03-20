# Control Quadrupeds Soft Contacts

Control of quadrupedal robots with soft contact constraints.


## Table of Contents

- [Installation with Docker](#installation-with-docker)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Known Bugs](#known-bugs)
- [Author](#author)


## Installation with Docker

Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine) with post-installation steps for Linux.

Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (nvidia-docker2) for NVIDIA support in the container. In order not to use NVIDIA, edit the Docker image and the `run.bash` script, removing the envs for NVIDIA support. In addition, remove the `additional_env` from the Gazebo process in `robot_launch/launch/robot.launch.py`.

Clone the repo:
```shell
git clone --recursive https://github.com/ddebenedittis/control_quadrupeds_soft_contacts
```
Build the docker image (-r option to rebuild the underlying images) (-l to install all the dependencies to use plot; the resulting image is bigger):
```shell
./build.bash [-r] [-l]
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
- `ROS2 Humble`, and the following ROS2 packages: `ros2-control`, `ros2-controllers`, `gazebo-ros-pkgs`, `gazebo-ros2-control`, `xacro`, `joint-state-publisher`, `joint-state-publisher-gui`
- [`Pinocchio`](https://github.com/stack-of-tasks/pinocchio)
- `numpy`, `scipy`, `numpy_quaternion`, [`quadprog`](https://github.com/quadprog/quadprog)


## Installation

```shell
git clone --recursive https://github.com/ddebenedittis/Quadruped_Control_Soft_Contacts Quadruped_Control_Soft_Contacts/src
cd Quadruped_Control_Soft_Contacts
colcon build --symlink-install
```


## Usage

### Simulations

- ANYmal C simulation:
```shell
ros2 launch robot_gazebo anymal_c.launch.py [terrain:={rigid,soft,very_soft,multi_terrains}] [gait:={teleop_base,static_walk,walking_trot,teleop_walking_trot}] [use_rviz:={False,True}] [save_csv:={False,True}] [reset:={False,True}]
```

![](https://raw.githubusercontent.com/ddebenedittis/media/main/control_quadrupeds_soft_contacts/gif/anymal_c-trot-grass.gif)

- SOLO12 static walk simulation:
```shell
ros2 launch robot_gazebo solo12.launch.py [terrain:={rigid,soft,very_soft,multi_terrains}] [save_csv:={False,True}] [reset:={False,True}]
```

![](https://raw.githubusercontent.com/ddebenedittis/media/main/control_quadrupeds_soft_contacts/gif/solo12-walk-rigid.gif)

- ANYmal C with SoftFoot-Qs static walk simulation:
```shell
ros2 launch robot_gazebo anymal_c_softfoot_q.launch.py [terrain:={rigid,soft,very_soft,multi_terrains}] [save_csv:={False,True}] [reset:={False,True}]
```

Optional arguments:
- `gait`: select the gait type. With `teleop_base` the base pose can be teleoperated without moving the feet, `static_walk` implements a static walk, and `walking_trot` implements a dynamic gait. Currently, the walking trot gait is implemented only on the ANYmal C robot. With `teleop_walking_trot` the robot is teleoperated by giving it a velocity command.
- `terrain`: select the terrain type: a rigid terrain (the gray one) or a soft terrain (the grass-like one).
- `reset`: must be used in another terminal when there is an already running simulation. The simulation will be restarted. The time is not reset to avoid problems with the controllers.
- `save_csv`: when `True`, some data is logged and saved in several .csv files in the `log/csv/` folder. This data can be plotted with `plot.py` (in the `logger_gazebo` package).
- `use_rviz`: when `True`, RViz is used to display the contact forces of the robot (still in Alpha version).

<img src="https://raw.githubusercontent.com/ddebenedittis/media/main/control_quadrupeds_soft_contacts/gif/rviz_legged_walk.gif" width="500">
<img src="https://raw.githubusercontent.com/ddebenedittis/media/main/control_quadrupeds_soft_contacts/gif/rviz_legged_trot.gif" width="500">

- `contact_constraint_type`: not working. Must be changed directly in the specific robot config .yaml file.

In some simulations (e.g. the simulation of ANYmal with SoftFoot-Q or the simulation with a trotting gait), due to the relatively high computational cost, it may be helpful to start or to run the simulation with a reduced real-time factor.

### Plot

- Plot and save the figures in `log/svg/` or `log/pdf/`:
```shell
ros2 run logger_gazebo plot
```

To save the plots, it is necessary to have a Docker image with latex installed. The image must have been built with the `-l` option.


### Add a new robot model

Add a new robot model in the `all_robots.yaml` file located in `src/robot/robot_model/robots/`. Pay attention to the order of the feet' names (should be Left Front, Right Front, Left Hind, Right Hind).

Add a new robot description package. It is recommended to place it in `src/robot/robots/`. The package should have both a .urdf file and a .xacro file, which needs to be augmented with the necessary ros2 plugins etc (see, for example, `anymal_c_simple_description/urdf/anymal_gazebo.xacro`).

Generate a new launch file similar to the ones already present in `src/robot/robot_gazebo/`.

Create a new `effort_controller.yaml` file, similar to the ones already present in `robot_control/config`. Edit at least the `robot_name` field and the `joints` fields (according to the names of the joints of your robot).


## Known Bugs

- Sometimes, when the `soft_sim` contact constraint is used in the controller's .yaml files, the simulation stops instantly.
- The controller does not work when the orientation is substantially different from zero roll, pitch, and yaw.


## Author

[Davide De Benedittis](https://3.bp.blogspot.com/-xvFfjYBPegM/VvFp02nHUjI/AAAAAAAAIoc/Mysj-ESrXPQFQI_yOJFQQz2kwZuIQiAKA/s1600/He-Man.png)