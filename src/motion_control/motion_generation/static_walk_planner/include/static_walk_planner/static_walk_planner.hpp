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


/// @class @brief 
class StaticWalkPlanner {
    public:
        StaticWalkPlanner();

        /// @brief Template for a planner. Outputs a constant pose.
        /// @param[out] gen_pose 
        void step_template(GeneralizedPose& /*gen_pose*/);

        /// @brief Step of a planner that makes the robot raise a foot.
        /// @param[out] gen_pose 
        void step_raise_foot(GeneralizedPose& /*gen_pose*/);

        /// @brief Step of a planner that implement a simple static walk.
        /// @param[out] gen_pose 
        void step(GeneralizedPose& gen_pose);

        /// @brief
        void reset() {
            phi_ = 0;
            init_com_position_.setZero();
        };

        void reset(Eigen::Vector3d init_com_position) {
            phi_ = 0;
            this->init_com_position_ = init_com_position;
        }

        /// @brief Planner sample time
        double dt_ = 1 / 200;

        /// @brief Initialization phase duration, expressed in full gait phase duration
        double init_phase_ = 0.2;

        /// @brief Names of the robot feet
        std::vector<std::string> all_feet_names_ = {"LF", "RF", "LH", "RH"};

        /// @brief Gait pattern sequence
        std::vector<std::string> gait_pattern_ = {"LF", "RH", "RF", "LH"};

        /// @brief Time period between two consecutive footfalls of the same foot (eg: LF)
        double cycle_duration_ = 4;

        /// @brief Time ratio of contact phase vs swing phase of a foot during its phase.
        double step_duty_factor_ = 0.7;

        /// @brief Lenght of a single step
        double step_length_ = 0.14;

        /// @brief Maximum height of a step
        double step_height_ = 0.1;

        /// @brief Desired foot penetration
        double desired_foot_penetration_ = 0.01;

        /// @brief Spline order for the generation of the swing feet trajectories
        int spline_order_ = 3;

        /// @brief Desired base height
        double h_base_des_ = 0.47;

        /// @brief Initial base height
        double h_base_init = 0.6;

        /// @brief Time normalized stride phase
        double phi_ = 0;

        /// @brief Coordinates x and y of the center of mass at the start of the gait cycle
        Eigen::Vector3d init_com_position_ = {0, 0, 0};

        /// @brief Absolute x and y coordinates of the feet relative to the COM in body frame. (The legs are symmetric)
        std::pair<double, double> abs_leg_pos_ = {0.4, 0.3};

        /// @brief Base oscillation quantities
        std::pair<double, double> base_osc_ = {0.06, 0.06};

    private:
        /// @brief Initialization phase 0: move the robot base from the initial position to a comfortable standing position.
        /// @param[out] gen_pose 
        void step_initialization_0(GeneralizedPose& gen_pose);

        /// @brief Initialization phase 1: move the robot base from the comfortable standing position to a new position so that the COG will lie in the next support polygon with a sufficient margin.
        /// @param[out] gen_pose 
        void step_initialization_1(GeneralizedPose& gen_pose);

        /// @brief Perform the real planner computations. 
        /// @param[out] gen_pose 
        void step_plan(GeneralizedPose& gen_pose);

        ///@brief 
        ///
        ///@param[in]  p_i 
        ///@param[in]  p_f 
        ///@param[in]  t 
        ///@param[out] p_t 
        ///@param[out] v_t 
        ///@param[out] a_t 
        void spline(
            Eigen::Ref<Eigen::Vector3d> p_i, Eigen::Ref<Eigen::Vector3d> p_f, double t,
            Eigen::Ref<Eigen::VectorXd> p_t, Eigen::Ref<Eigen::VectorXd> v_t, Eigen::Ref<Eigen::VectorXd> a_t
        );
};

} // namespace static_walk_planner