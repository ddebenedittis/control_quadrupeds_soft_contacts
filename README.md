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

- ANYmal C static walk simulation:
```shell
ros2 launch robot_gazebo anymal.launch.py [terrain_type:={rigid,soft_mattress}] [save_csv:={False,True}] [reset:={False,True}]
```
- SOLO12 static walk simulation:
```shell
ros2 launch robot_gazebo solo.launch.py [terrain_type:={rigid,soft_mattress}] [save_csv:={False,True}] [reset:={False,True}]
```
- ANYmal C with SoftFoot-Qs static walk simulation:
```shell
ros2 launch robot_gazebo anymal_c_softfoot_q.launch.py [terrain_type:={rigid,soft_mattress}] [save_csv:={False,True}] [reset:={False,True}]
```

With the `terrain_type` parameter, the robot walks on a soft and deformable mattress.

With `save_csv:=True`, some data is logged and saved in several .csv files in the `log` folder. This data can be plotted with `plot.py` in the `logger_gazebo` package.

The `reset:=True` parameter must be used in another terminal when there is an already running simulation. The simulation will be restarted. The time is not reset to avoid problems with the controllers.

Other parameters, such as `contact_constraint_type: {soft_kv, rigid}`, must be changed directly in the specific robot config .yaml file.


### Add a new robot model

Add a new robot model in the `all_robots.yaml` file located in `src/robot/robot_model/robots/`. Pay attention to the order of the feet' names (should be Left Front, Right Front, Left Hind, Right Hind).

Add a new robot description package. It is recommended to place it in `src/robot/robots/`. The package should have both a .urdf file and a .xacro file, which needs to be augmented with the necessary ros2 plugins etc (see, for example, `anymal_c_simple_description/urdf/anymal_gazebo.xacro`).

Generate a new launch file similar to the ones already present in `src/robot/robot_gazebo/`.

Create a new `effort_controller.yaml` file, similar to the ones already present in the other robot packages. Edit at least the `robot_name` field and the `joints` fields (according to the names of the joints of your robot).


## Author

[Davide De Benedittis](https://3.bp.blogspot.com/-xvFfjYBPegM/VvFp02nHUjI/AAAAAAAAIoc/Mysj-ESrXPQFQI_yOJFQQz2kwZuIQiAKA/s1600/He-Man.png)