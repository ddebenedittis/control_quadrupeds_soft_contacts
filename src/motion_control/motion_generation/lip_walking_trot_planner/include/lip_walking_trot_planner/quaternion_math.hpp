#pragma once 

#include "Eigen/Core"
#include "Eigen/Geometry"



namespace lip_walking_trot_planner {

using namespace Eigen;

Quaterniond compute_quaternion_from_euler_angles(double roll, double pitch, double yaw);

Quaterniond quat_mult(const Quaterniond& q1, const Quaterniond& q2);

void quat_rot(const Quaterniond& quat, Vector3d& vec);

} // lip_walking_trot_planner