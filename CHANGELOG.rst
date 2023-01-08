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