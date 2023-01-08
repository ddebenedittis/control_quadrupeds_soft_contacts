#pragma once

#include "pinocchio/algorithm/kinematics.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <string>
#include <vector>



namespace robot_wrapper {


/// @class @brief Creates the robot model from an urdf file and performs dynamics and kinematics computations.
/// 
/// @details The class constructs the robot model by loading the informations relative to the input robot stored in the file all_robots.yaml.
/// Using these informations it finds the urdf model of the robot and constructs the model and data using Pinocchio.
/// Then, using Pinocchio, it computes some relevant kinematics and dynamics quantities.
class RobotModel {
    public:
        /// @brief Construct a new Robot Model object by loading the robot_name robot.
        /// @param[in] robot_name
        RobotModel(const std::string& robot_name);

        /// @brief Update the joints and frame placements, and compute M and h.
        /// @param[in] q
        /// @param[in] v
        /// @attention This function must be called before performing any of the kinematics and dynamics computations.
        void compute_EOM(const Eigen::VectorXd& q, const Eigen::VectorXd& v);

        /// @brief Update the joints accelerations (for computing the various J_dot v).
        /// @param[in] q
        /// @param[in] v
        /// @attention This function must be called before calling the various J_i_dot_times_v.
        void compute_second_order_FK(const Eigen::VectorXd& q, const Eigen::VectorXd& v);

        /// @brief Get the stack of the contact jacobians Jc.
        /// @param[out] Jc [3*nc, nv]
        /// @warning Compute_EOM must have been previously called.
        void get_Jc(Eigen::MatrixXd& Jc);

        /// @brief Get the jacobian of the base of the robot.
        /// @param[out] Jb [3, nv]
        /// @warning Compute_EOM must have been previously called.
        void get_Jb(Eigen::MatrixXd& Jb);

        /// @brief Get the stack of the jacobian of the feet in swing phase.
        /// @param[out] Js [3*(n_feet-nc), nv]
        /// @warning Compute_EOM must have been previously called.
        void get_Js(Eigen::MatrixXd& Js);

        /// @brief Get the Jc_dot * v vector.
        /// @param Jc_dot_times_v [3*nc]
        /// @warning Compute_second_order_FK must have been previously called.
        void get_Jc_dot_times_v(Eigen::VectorXd& Jc_dot_times_v);

        /// @brief Get the Jb_dot * v vector.
        /// @param Jb_dot_times_v [6]
        /// @warning Compute_second_order_FK must have been previously called.
        void get_Jb_dot_times_v(Eigen::VectorXd& Jb_dot_times_v);

        /// @brief Get the Js_dot * v vector.
        /// @param Js_dot_times_v [3*(n_feet-nc)]
        /// @warning Compute_second_order_FK must have been previously called.
        void get_Js_dot_times_v(Eigen::VectorXd& Js_dot_times_v);

        /// @brief Get the rotation matrix oRb.
        /// @param[out] oRb [3, 3]
        /// @warning Compute_EOM must have been previously called.
        void get_oRb(Eigen::Matrix3d& oRb);

        /// @brief Get the positions of the feet in swing phase.
        /// @param[out] r_s [3*(n_feet-nc)]
        /// @warning Compute_EOM must have been previously called.
        void get_r_s(Eigen::VectorXd& r_s);

        /// @brief
        /// @warning Compute_EOM must have been previously called.
        Eigen::VectorXd get_feet_position();

        pinocchio::Model& get_model() { return model; }
        
        pinocchio::Data& get_data() { return data; }

        const std::vector<std::string>& get_generic_feet_names() const {return generic_feet_names;}

        const std::vector<std::string>& get_all_feet_names() const {return feet_names;}

        void set_feet_names(const std::vector<std::string>& contact_feet_names) {
            this->contact_feet_names = generic_to_specific_feet_names(contact_feet_names);

            set_swing_feet_names();
        }

        std::vector<std::string> generic_to_specific_feet_names(std::vector<std::string> generic) {
            for (int i=0; i<static_cast<int>(generic.size()); i++) {
                auto it = std::find(this->generic_feet_names.begin(), this->generic_feet_names.end(), generic[i]);

                int index = std::distance(this->generic_feet_names.begin(), it);

                generic[i] = this->feet_names[index];
            }

            return generic;
        }




    /* ====================================================================== */

    private:
        /// @brief Compute the names of the feet in swing phase.
        /// @details The feet in swing phase are all and only the feet not in contact with the terrain.
        void set_swing_feet_names();

        pinocchio::Model model;
        
        pinocchio::Data data;

        std::string urdf_path;

        /// @brief
        std::vector<std::string> generic_feet_names = {"LF", "RF", "LH", "RH"};

        /// @brief The names of all the robot feet.
        std::vector<std::string> feet_names;

        /// @brief The names of the robot feet in contact with the terrain.
        std::vector<std::string> contact_feet_names;
        
        /// @brief The names of the robot feet in swing phase.
        std::vector<std::string> swing_feet_names;

        /// @brief The position of the feet contact point with the terrain relative to the position of the feet frame, in inertial frame. 
        /// @details The position of the foot link computed from the robot model is not necessarly equal to the expected position of the point of contact with the terrain.
        Eigen::VectorXd feet_displacement;
};

} // robot_wrapper