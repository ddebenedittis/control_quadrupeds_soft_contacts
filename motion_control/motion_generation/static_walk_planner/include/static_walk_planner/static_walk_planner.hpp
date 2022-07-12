#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace static_walk_planner {


/* ========================================================================== */
/*                           GENERALIZEDPOSE STRUCT                           */
/* ========================================================================== */

// TODO: this same struct is defined here and in whole_body_controller/prioritized_tasks.hpp. Avoid this repetition.

/// @brief Stores the generalized desired pose computed by a planner and used by the whole-body controller.
struct GeneralizedPose {
    // Base linear quantities
    Eigen::Vector3d base_acc = {0, 0, 0};
    Eigen::Vector3d base_vel = {0, 0, 0};
    Eigen::Vector3d base_pos = {0, 0, 0};

    // Base angular quantities
    Eigen::Vector3d base_angvel = {0, 0, 0};
    Eigen::Vector4d base_quat = {0, 0, 0, 1};

    // Swing feet linear quantities
    Eigen::VectorXd feet_acc = {};
    Eigen::VectorXd feet_vel = {};
    Eigen::VectorXd feet_pos = {};

    // List of feet names in contact with the ground
    std::vector<std::string> contact_feet_names;
};



class StaticWalkPlanner {
    public:
        StaticWalkPlanner();

        GeneralizedPose plan();

    private:
        ///@brief 
        ///
        ///@param[in]  p_i 
        ///@param[in]  p_f 
        ///@param[in]  t 
        ///@param[out] p_t 
        ///@param[out] v_t 
        ///@param[out] a_t 
        void spline(
            Eigen::VectorXd p_i, Eigen::VectorXd p_f, double t,
            Eigen::VectorXd p_t, Eigen::VectorXd v_t, Eigen::VectorXd a_t
        );

        /// @brief Gait pattern sequence
        std::vector<std::string> gait_pattern_ = {"LF", "RH", "RF", "LH"};

        /// @brief Time period between two consecutive footfalls of the same foot (eg: LF)
        double cycle_duration_ = 4;

        /// @brief Ratio in which the designated swing foot is in contact with the terrain in order to move the COM position
        double step_duty_factor_ = 0.7;

        /// @brief Lenght of a single step
        double step_length_ = 0.14;

        /// @brief Maximum height of a step
        double step_height_ = 0.1;

        /// @brief Spline order for the generation of the swing feet trajectories
        int spline_order_ = 3;

        /// @brief Desired base height
        double h_base_des_ = 0.47;

        /// @brief Time normalized stride phase
        double phi_ = 0;

        /// @brief Coordinates x and y of the center of mass at the start of the gait cycle
        std::pair<double, double> init_com_position_ = {0, 0};

        /// @brief Absolute x and y coordinates of the feet relative to the COM in body frame. (The legs are symmetric)
        std::pair<double, double> abs_leg_pos_ = {0.4, 0.3};

        /// @brief Base oscillation quantities
        std::pair<double, double> base_osc = {0.06, 0.06};
};

} // namespace static_walk_planner