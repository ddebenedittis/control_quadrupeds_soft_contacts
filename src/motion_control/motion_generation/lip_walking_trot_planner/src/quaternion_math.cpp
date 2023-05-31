#include "lip_walking_trot_planner/quaternion_math.hpp"



namespace lip_walking_trot_planner {

using namespace Eigen;


Quaterniond compute_quaternion_from_euler_angles(double roll, double pitch, double yaw)
{
    double q_x = std::sin(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) - std::cos(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
    double q_y = std::cos(roll/2) * std::sin(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::cos(pitch/2) * std::sin(yaw/2);
    double q_z = std::cos(roll/2) * std::cos(pitch/2) * std::sin(yaw/2) - std::sin(roll/2) * std::sin(pitch/2) * std::cos(yaw/2);
    double q_w = std::cos(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);

    return {q_w, q_x, q_y, q_z};
}


void quat_rot(const Quaterniond& quat, Vector3d& vec)
{
    Quaterniond vec_quat = Quaterniond(0, vec[0], vec[1], vec[2]);

    Quaternion vec_out = quat * vec_quat * quat.conjugate();

    vec[0] = vec_out.x();
    vec[1] = vec_out.y();
    vec[2] = vec_out.z();
}

} // lip_walking_trot_planner