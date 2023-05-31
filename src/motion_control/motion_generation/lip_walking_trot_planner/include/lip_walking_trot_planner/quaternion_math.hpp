#pragma once 

#include "Eigen/Core"
#include "Eigen/Geometry"



namespace lip_walking_trot_planner {

using namespace Eigen;

Quaterniond compute_quaternion_from_euler_angles(double roll, double pitch, double yaw);

/// @brief Rotate the input vector vec with quat: vec = q * vec * q.conj.
/// 
/// @param[in] quat 
/// @param[in,out] vec 
void quat_rot(const Quaterniond& quat, Vector3d& vec);

} // lip_walking_trot_planner