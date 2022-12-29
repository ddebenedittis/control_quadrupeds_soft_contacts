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