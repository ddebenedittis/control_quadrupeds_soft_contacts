#pragma once

#include "generalized_pose_msgs/msg/generalized_pose.hpp"

#include <iostream>
#include <string>
#include <vector>


namespace generalized_pose {



/* ========================================================================== */
/*                                   VECTOR3                                  */
/* ========================================================================== */

class Vector3 {
public:
    Vector3() = default;

    Vector3(double x, double y, double z)
    : x(x),
      y(y),
      z(z) {}

    template <typename T> Vector3(const T vector)
    : x(*(vector.begin())),
      y(*(vector.begin()+1)),
      z(*(vector.begin()+2))
    {
        // static_assert(vector.size() == 3, "The input argument does not have three elements.");
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector3& vec)
    {
        os << "[" << vec.x << ", " << vec.y << ", " << vec.z << "]";

        return os;
    }
    
    static geometry_msgs::msg::Vector3 get_vector3_msg(const Vector3& v)
    {
        geometry_msgs::msg::Vector3 ret;

        ret.x = v.x;
        ret.y = v.y;
        ret.z = v.z;

        return ret;
    }

    double x = 0;
    double y = 0;
    double z = 0;
};



/* ========================================================================== */
/*                                 QUATERNION                                 */
/* ========================================================================== */

class Quaternion {
public:
    Quaternion() = default;

    Quaternion(double x, double y, double z, double w)
    : x(x),
      y(y),
      z(z),
      w(w) {}

    template <typename T> Quaternion(const T& quat)
    : x(quat.x),
      y(quat.y),
      z(quat.z),
      w(quat.w)
    {
        // static_assert(quat.size() == 4, "The input argument does not have four elements.");
    }

    friend std::ostream& operator<<(std::ostream& os, const Quaternion& quat)
    {
        os << "[x: " << quat.x << ", y: " << quat.y << ", z: " << quat.z << ", w: " << quat.w << "]";

        return os;
    }

    static geometry_msgs::msg::Quaternion get_quaternion_msg(const Quaternion& q)
    {
        geometry_msgs::msg::Quaternion ret;

        ret.x = q.x;
        ret.y = q.y;
        ret.z = q.z;
        ret.z = q.w;

        return ret;
    }

    /* ====================================================================== */

    double x = 0;
    double y = 0;
    double z = 0;
    double w = 1;
};



/* ========================================================================== */
/*                            GENERALIZEDPOSESTRUCT                           */
/* ========================================================================== */

/// @brief Stores the generalized desired pose computed by a planner and used by the whole-body controller.
class GeneralizedPoseStruct {
public:
    /// @brief Default constuctor
    GeneralizedPoseStruct() = default;

    /// @brief Constructor
    template <typename Vec3, typename Quat, typename VecX>
    GeneralizedPoseStruct(
        const Vec3& base_acc, const Vec3& base_vel, const Vec3& base_pos,
        const Vec3& base_angvel, const Quat& base_quat,
        const VecX& feet_acc, const VecX& feet_vel, const VecX& feet_pos,
        const std::vector<std::string>& contact_feet_names)
    : base_acc(base_acc),
      base_vel(base_vel),
      base_pos(base_pos),
      base_angvel(base_angvel),
      base_quat(base_quat),
      feet_acc(feet_acc.data(), feet_acc.data() + feet_acc.size()),
      feet_vel(feet_vel.data(), feet_vel.data() + feet_vel.size()),
      feet_pos(feet_pos.data(), feet_pos.data() + feet_pos.size()),
      contact_feet_names(contact_feet_names)
    {
        // static_assert((feet_acc.size == feet_vel.size() == feet_pos.size()), "The swing feet quantities do not have the same dimension.");
    }

    /// @brief Constructor
    template <typename Vec3, typename Quat>
    GeneralizedPoseStruct(
        const Vec3& base_pos,
        const Quat& base_quat)
    : base_pos(base_pos),
      base_quat(base_quat)
    {
        // static_assert((feet_acc.size == feet_vel.size() == feet_pos.size()), "The swing feet quantities do not have the same dimension.");
    }

    friend std::ostream& operator<<(std::ostream& os, const GeneralizedPoseStruct& gen_pose)
    {
        os << "base acc:           " << gen_pose.base_acc << "\n"
           << "base vel:           " << gen_pose.base_vel << "\n"
           << "base pos:           " << gen_pose.base_pos << "\n"
           << "base ang vel:       " << gen_pose.base_angvel << "\n"
           << "base quat:          " << gen_pose.base_quat << "\n"
           << "feet_pos:           [";

        for (int i = 0; i < gen_pose.feet_pos.size(); i+=3) {
            os << "[" << gen_pose.feet_pos[i] << ", "
                      << gen_pose.feet_pos[i+1] << ", "
                      << gen_pose.feet_pos[i+2] << "] ";
        }
        os << "]\n";

        os << "contact feet names: [";
        for (auto& e : gen_pose.contact_feet_names) {
            os << e << " ";
        }
        os << "]";

        return os;
    }

    /// @brief Get the corresponding ROS message
    [[nodiscard]] generalized_pose_msgs::msg::GeneralizedPose get_msg() const
    {
        generalized_pose_msgs::msg::GeneralizedPose gen_pose;

        gen_pose.base_acc = get_vector3_msg(base_acc);
        gen_pose.base_vel = get_vector3_msg(base_vel);
        gen_pose.base_pos = get_vector3_msg(base_pos);

        gen_pose.base_angvel = get_vector3_msg(base_angvel);
        gen_pose.base_quat = get_quaternion_msg(base_quat);

        gen_pose.feet_acc = feet_acc;
        gen_pose.feet_vel = feet_vel;
        gen_pose.feet_pos = feet_pos;

        gen_pose.contact_feet = contact_feet_names;

        return gen_pose;
    }

    /* ====================================================================== */

    // Base linear quantities
    Vector3 base_acc = {0, 0, 0};
    Vector3 base_vel = {0, 0, 0};
    Vector3 base_pos = {0, 0, 0};

    // Base angular quantities
    Vector3 base_angvel = {0, 0, 0};
    Quaternion base_quat = {0, 0, 0, 1};

    // Swing feet linear quantities
    std::vector<double> feet_acc = {};
    std::vector<double> feet_vel = {};
    std::vector<double> feet_pos = {};

    // List of feet names in contact with the ground
    std::vector<std::string> contact_feet_names = {
        "LF", "RF", "LH", "RH"
    };

private:
    static geometry_msgs::msg::Vector3 get_vector3_msg(const Vector3& v)
    {
        geometry_msgs::msg::Vector3 ret;

        ret.x = v.x;
        ret.y = v.y;
        ret.z = v.z;

        return ret;
    }

    static geometry_msgs::msg::Quaternion get_quaternion_msg(const Quaternion& q)
    {
        geometry_msgs::msg::Quaternion ret;

        ret.x = q.x;
        ret.y = q.y;
        ret.z = q.z;
        ret.w = q.w;

        return ret;
    }
};

} // generalized_pose