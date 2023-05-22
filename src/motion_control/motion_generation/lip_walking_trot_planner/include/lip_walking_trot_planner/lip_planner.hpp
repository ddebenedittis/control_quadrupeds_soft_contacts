#pragma once

#include "lip_walking_trot_planner/interpolator.hpp"

#include "generalized_pose_msgs/generalized_pose_struct.hpp"

#include <Eigen/Geometry>



namespace lip_walking_trot_planner
{

using namespace Eigen;



/* ========================================================================== */
/*                                MOTIONPLANNER                               */
/* ========================================================================== */

class MotionPlanner {
public:
    MotionPlanner();

    void update_initial_conditions(
        const Ref<Vector3d>& p_0 = (Vector3d() << 0, 0, 0).finished(),
        double yaw = 0
    );

    generalized_pose::GeneralizedPoseStruct update(
        const Ref<Vector3d>& p_com, const Ref<Vector3d>& v_com, const Ref<Vector3d>& a_com,
        const Ref<Vector2d>& vel_cmd, double yaw_rate_cmd
    );

    static std::tuple<Vector3d, Vector3d, Vector3d> spline(
        const Vector3d& p_i, const Vector3d& p_f, double phi, InterpolationMethod method
    ) {
        return Interpolator::spline<Vector3d>(p_i, p_f, phi, method);
    };

    [[nodiscard]] double get_dtheta() const {return _dtheta;}

    [[nodiscard]] double get_height_com() const {return _height_com;}

    [[nodiscard]] double get_sample_time() const {return _dt;}

private:
    static MatrixXd compute_A_t(double omega, double t);
    static MatrixXd compute_b_t(double omega, double t);

    double mpc_qp(
        const Vector2d& Xcom_0, double p_0,
        double xcom_dot_des
    );

    std::vector<Vector2d> compute_desired_footholds(const Vector2d& p_star);

    std::tuple<VectorXd, VectorXd, VectorXd> compute_foot_trajectory(
        const std::vector<Vector2d>& pos_swing_feet
    );

    void switch_swing_feet(const std::vector<Vector2d>& p_swing_feet);

    std::tuple<Vector3d, Vector3d, Vector3d> get_des_base_pose(
        const Vector2d& Xcom_0, const Vector2d& Ycom_0,
        double px_0, double py_0
    );

    generalized_pose::GeneralizedPoseStruct mpc(
        const Vector3d& p_com, const Vector3d& v_com, const Vector3d& a_com,
        const Vector2d& vel_cmd, double yaw_rate_cmd
    );

    /// @brief Return true when the robot should stop moving.
    /// @details True when the commanded linear and angular velocity has been equal to 0 for a number of robot steps grater than _max_fixed_steps and the previous swing phase has finished.
    bool check_stop(const Vector2d& vel_cmd, double yaw_rate_cmd);

    [[nodiscard]] std::vector<std::vector<Vector3d>> compute_trajectory_sample_points();

    /// @brief Number of future steps over which the optimization is performed
    int _n = 3;

    // Cost function matrices
    // cost = sum 1/2 [ (xcom_dot - xcom_dot_des)^T Q (xcom_dot - xcom_dot_des) + (px - px_m)^T R (px - px_m) ]
    MatrixXd _Q = 1000 * MatrixXd::Identity(_n, _n);
    MatrixXd _R = 1 * MatrixXd::Identity(_n, _n);

    /// @brief Planner time step
    double _dt = 1. / 200.;

    /// @brief Time normalized stride phase
    double _phi = 0.;

    /// @brief Swing feet names
    std::vector<std::string> _swing_feet_names = {"LF", "RH"};

    /// @brief Kinematic reachability limit for the stance feet
    double _step_reachability = 0.2;

    // Parameters used to describe the feet position with respect to the ZMP
    double _theta_0 = 0.64;
    double _r = 0.5;

    /// @brief Commanded yaw angle
    double _dtheta = 0.;

    Interpolator _interpolator = Interpolator(
        InterpolationMethod::Spline_5th, 0.2,
        0.0, 0.1, -0.025
    );

    /// @brief Initial swing feet positions at the start of the swing phase.
    std::vector<Vector3d> _init_pos_swing_feet = {};

    /// @brief Desired foothold positions at the end of the swing phase.
    std::vector<Vector2d> _final_pos_swing_feet = {};

    /// @brief Desired height of the center of mass
    double _height_com = 0.5;

    // The planner stops moving the feet when the reference velocity (linear and angular) has been equal to zero for a number of steps equal to max_fixed_steps.
    int _max_fixed_steps = 6;
    int _fixed_steps = 6;

    /// @brief Short ordered names of the swing feet.
    std::vector<std::string> _all_feet_names = {"LF", "RF", "LH", "RH"};

    /// Number of samples of a swing trajectory computed by compute_trajectory_sample_points().
    int _n_sample_points = 10;
};

}