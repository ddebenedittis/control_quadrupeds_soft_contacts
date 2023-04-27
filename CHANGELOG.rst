0.1.0 (2022-12-23)
------------------
- Updated to ROS 2 Humble.
- Corrected some bugs in the Kalman filter.
- More uniform package.xml files.
- Minor submodules update.

0.1.1 (2022-12-29)
------------------
- Added new ros packages to the dockerfile: joint_state_publisher and joint_state_publisher_gui. They are used to display the robot with RViz.
- Bug fix: run.bash did not work because of the comments.
- Bug fix: run.bash updated so that running the container does not raise warnings.
- run.bash and build.bash updated to use variables.
- Fixed some minor warnings in hqp_controller and robot_model.
- Submodules update. Fixed some warnings in quadprog. Updated anymal_c_simple_description and solo_description to have working launch.py files. Updated their CMakeLists.txt, package.xml, and README.md.
- New submodules: anymal_c_softfoot_q_description and softfoot_thing. They allow the robot 
- Added some missing dependencies to some packages.

0.1.2 (2023-01-08)
------------------
- Added support for ANYmal C with SoftFeet-Q.
- The motion planner always uses the following feet names: LF RF LH RH. It is robot_model that receives these generic names and convert them to the specific feet names.
- Removed cycles_completed_ from the static_walk_planner, init_com_position_ is used instead.
- Added plugins that publish the contact forces and the feet positions to the robot models.
- Can choose between a soft contact model and a rigid contact model. The terminal argument does not work, the specific robot .yaml config file must be edited (contact_constraint_type parameter).
- Can spawn a soft mattress under the robot.
- Created a logging package, that saves a csv of the history some relevant quantities. For this, hqp_controller must publish some messages (set logging to true in the .yaml config file of the specific robot).
- The simulation can be restarted without closing gazebo now.
- Corrected some bugs

0.1.3 (2023-02-14)
------------------
- Moved the config .yaml files from the respective robot_description package to the robot_control package.
- Modified the contact model (both the rigid and the soft contact model):
  Jc u_dot + Jc_dot u = - k_c_v Jc u + ... 
  This term is proportional to the velocity of the contact point.
- The hierarchical_qp prints an error when the constraints are inconsistent or when the G matrix is not positive definite.
- Implemented the soft_sim contact model in the controller.
  This is the contact model used in most physics engines (e.g. ODE and DART). It models the contact with a spring and damper in parallel and normal to the contact direction. Along the tangential directions, the rigid contact model is used (no slippage).
  The controller is adapted to deal with deformations of different size.
- The controller interpolates the robot configuration from q0 to qi (given in the config file).
- The soft mattress has been substituted by a soft terrain.
- Better data logging and plotting.
- Solo 12 now has contact sensors.
- Shank collision removed from ANYmal C in order to avoid double collisions with soft terrains.
- Trotting works. But its parameters are suboptimal.

0.1.4 (2023-03-21)
------------------
- New submodule: visualizer with RViz. The robot, the terrain, and the (optimal) contact forces are displayed. The support polygon is displayed. The friction cones are displayed. The COM (center of mass) and the ZMP (zero moment point) are displayed. The feet trajectories are displayed.
- Fixed the teleoperate_robot_base node. New teleoperate_velocity_command node that teleoperates the robot trotting (use `gait:=teleop_walking_trot`).
- Reorganized the gazebo launch files.
- Added an heterogeneous terrain (multi_terrains).
- New parameter in the controllers yaml file: `shift_base_height`. When `true`, we consider the fact that the terrain is soft and move the desired base height to take into account the terrain penetration. Currently deprecated, due to the use of the terrain height.
- New task that enforces limits on the knee joints coordinates.
- The `terrain_estimator` package gives a very rough estimate of the contact plane. The local terrain height is used by the trotting planner to adjust the commanded height.
- Important bug fix in the whole-body controller. Previously, it did not work when the orientation was very different from zero roll, pitch, and yaw.

Upcoming (2023-04-26)
------------------
- Started using clang-tidy. The docker image installs ament-clang-tidy if the image is built with the `-d` option.
- The estimated robot pose is displayed in RViz.
- The state estimator works correctly and does not diverge. The filter gains are not set. The sensor noises are zero.
- The robot can be spawned in a terrain created from an heightmap.
- The robot measures the penetration with the terrain and increase the step height accordingly.
