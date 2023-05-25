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


Quaterniond quat_mult(const Quaterniond& q1, const Quaterniond& q2)
{
    double x1 = q1.x();
    double y1 = q1.y();
    double z1 = q1.z();
    double w1 = q1.w();

    double x2 = q2.x();
    double y2 = q2.y();
    double z2 = q2.z();
    double w2 = q2.w();

    double x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    double y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
    double z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;
    double w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;

    return {w, x, y, z};
}


void quat_rot(const Quaterniond& quat, Vector3d& vec)
{
    Quaterniond vec_quat = Quaterniond(0, vec[0], vec[1], vec[2]);

    Quaternion vec_out = quat_mult(quat_mult(quat, vec_quat), quat.conjugate());

    vec[0] = vec_out.x();
    vec[1] = vec_out.y();
    vec[2] = vec_out.z();
}

} // lip_walking_trot_planner