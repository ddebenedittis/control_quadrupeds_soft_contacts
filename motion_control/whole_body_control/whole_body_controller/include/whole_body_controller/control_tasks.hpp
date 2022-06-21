/* ========================================================================== */
/*                                 DESCRIPTION                                */
/* ========================================================================== */

/*

*/



/* ========================================================================== */
/*                              IMPORT LIBRARIES                              */
/* ========================================================================== */

#pragma once

#include "robot_model/robot_model.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>



/* ========================================================================== */
/*                                    CODE                                    */
/* ========================================================================== */

namespace wbc {

class ControlTasks {
    public:
        ControlTasks(std::string robot_name);

        /**
         * @brief Reset the class before restarting the optimization problem.
         * 
         * @param q 
         * @param v 
         * @param contact_feet_names 
         */
        void reset(Eigen::VectorXd& q, Eigen::VectorXd& v, std::vector<std::string> contact_feet_names);

        /**
         * @brief Compute the matrices A and b that enforce the dynamic consistency with the Equations of Motion of the floating base.
         * 
         * @param A 
         * @param b 
         */
        void task_floating_base_eom(Eigen::Ref<Eigen::MatrixXd>& A, Eigen::Ref<Eigen::VectorXd>& b);

        /**
         * @brief Compute the matrices C and d that enfore the limits on the joint torques.
         * 
         * @param C 
         * @param d 
         */
        void task_torque_limits(Eigen::Ref<Eigen::MatrixXd>& C, Eigen::Ref<Eigen::VectorXd>& d);

        /**
         * @brief Compute C and d that enforce the friction limits and the limits on the normal component of the contact forces.
         * 
         * @param C 
         * @param d 
         */
        void task_friction_Fc_modulation(Eigen::Ref<Eigen::MatrixXd>& C, Eigen::Ref<Eigen::VectorXd>& d);

        /**
         * @brief Compute
         * 
         * @param A 
         * @param b 
         * @param r_b_ddot_des 
         * @param r_b_dot_des 
         * @param r_b_des 
         */
        void task_linear_motion_tracking(
            Eigen::Ref<Eigen::MatrixXd>& A, Eigen::Ref<Eigen::VectorXd>& b,
            Eigen::VectorXd& r_b_ddot_des, Eigen::VectorXd& r_b_dot_des, Eigen::VectorXd& r_b_des
        );

        void task_angular_motion_tracking(
            Eigen::Ref<Eigen::MatrixXd>& A, Eigen::Ref<Eigen::VectorXd>& b,
            Eigen::Vector3d& omega_des, Eigen::Vector4d& q_des
        );

        void task_swing_feet_tracking(
            Eigen::Ref<Eigen::MatrixXd>& A, Eigen::Ref<Eigen::VectorXd>& b,
            Eigen::VectorXd& r_s_ddot_des, Eigen::VectorXd& r_s_dot_des, Eigen::VectorXd& r_s_des
        );

        void task_contact_constraints();

        void task_energy_forces_optimization(Eigen::Ref<Eigen::MatrixXd>& A, Eigen::Ref<Eigen::VectorXd>& b);

    private:
        robot_wrapper::RobotModel robot_model;

        Eigen::VectorXd q;
        Eigen::VectorXd v;

        double tau_max;
        double mu;
        double Fn_max;
        double Fn_min;

        int nv;
        int nc;
        int nc3;
        int ns3;

        Eigen::MatrixXd M;
        Eigen::VectorXd h;
        Eigen::MatrixXd Jc;

        Eigen::MatrixXd Jb;
        Eigen::VectorXd Jb_dot_times_v;

        Eigen::MatrixXd Kp_b_pos;
        Eigen::MatrixXd Kd_b_pos;

        Eigen::MatrixXd Kp_b_ang;
        Eigen::MatrixXd Kd_b_ang;

        Eigen::MatrixXd Kp_s_pos;
        Eigen::MatrixXd Kd_s_pos;
};

} // namespace wbc