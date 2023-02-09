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

upcoming
------------------
- Moved the config .yaml files from the respective robot_description package to the robot_control package.
- Fixed some minor warnings in the quadprog package.
- Added the terrain height to the planner parameters.
- Modified the contact model (both the rigid and the soft contact model):
  Jc u_dot + Jc_dot u = - k_c_v Jc u + ... 
  This term is proportional to the velocity of the contact point.
- The hierarchical_qp prints an error when the constraints are inconsistent or when the G matrix is not positive definite.
- Implemented the soft_sim contact model in the controller.
  This is the contact model used in most physics engines (e.g. ODE and DART). It models the contact with a spring and damper in parallel and normal to the contact direction. Along the tangential directions, the rigid contact model is used (no slippage).
  The controller is adapted to deal with deformations of different size.
- Reorganized prioritized_tasks. Now it is necessary to specify the dimension of the tasks only in one place.
- Updated CMakeLists.txt to C++17.
- Added a brief package description to some packages.
- The controller interpolates the robot configuration from q0 to qi (given in the config file).
- The soft mattress has been substituted by a soft terrain.
- More data logged with logger_gazebo and the script is better organized. The data can be plotted with the plot script directly in the docker image (which must be built with the `-l` option in this case).
- Solo 12 now has contact sensors.
- Shank collision removed from ANYmal C in order to avoid double collisions with soft terrains.
- The node for trotting works. But ANYmal falls.