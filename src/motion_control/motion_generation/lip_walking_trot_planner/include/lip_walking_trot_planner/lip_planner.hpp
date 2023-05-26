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
        double yaw = 0,
        const std::vector<Vector3d>& feet_positions = {}
    );

    generalized_pose::GeneralizedPoseStruct update(
        const Ref<Vector3d>& p_com, const Ref<Vector3d>& v_com, const Ref<Vector3d>& a_com,
        const Ref<Vector2d>& vel_cmd, double yaw_rate_cmd,
        const Vector3d& plane_coeffs = {0, 0, 0},
        const std::vector<Vector3d>& feet_positions = {}, const std::vector<Vector3d>& feet_velocities = {}
    );

    static std::tuple<Vector3d, Vector3d, Vector3d> spline(
        const Vector3d& p_i, const Vector3d& p_f, double phi, InterpolationMethod method
    ) {
        return Interpolator::spline<Vector3d>(p_i, p_f, phi, method);
    };

    [[nodiscard]] std::vector<std::vector<Vector3d>> compute_trajectory_sample_points() const;

    /* =============================== Setters ============================== */

    int set_sample_time(double dt)
    {
        if (dt > 0) {
            dt_ = dt;
        } else {
            std::cerr << "The sample time must be > 0" << std::endl;
            return 1;
        }

        return 0;
    }

    int set_interpolation_method(const std::string& method)
    {
        if (method == "spline3") {
            interpolator_.set_method(InterpolationMethod::Spline_3rd);
        } else if (method == "spline5") {
            interpolator_.set_method(InterpolationMethod::Spline_5th);
        } else if (method == "cycloid") {
            interpolator_.set_method(InterpolationMethod::Cycloid);
        } else {
            std::cerr << "In lip_walking_trot_planner::MotionPlanner.set_interpolation_method"
                "the interpolation method specified is not an acceptable value.\n"
                "It must be in [\"spline3\", \"spline5\", \"cycloid\"]" << std::endl;;

            return 1;
        }

        return 0;
    }

    int set_step_duration(double step_duration) {return interpolator_.set_step_duration(step_duration);}

    int set_step_horizontal_phase_delay(double horizontal_phase_delay) {return interpolator_.set_step_horizontal_phase_delay(horizontal_phase_delay);}

    int set_step_height(double step_height) {return interpolator_.set_step_height(step_height);}

    void set_foot_penetration(double foot_penetration) {return interpolator_.set_foot_penetration(foot_penetration);}

    void set_interpolate_swing_feet_from_current_position(bool flag) {interpolate_swing_feet_from_current_position_ = flag;}

    /* =============================== Getters ============================== */

    [[nodiscard]] double get_dtheta() const {return dtheta_;}

    [[nodiscard]] double get_height_com() const {return height_com_;}

    [[nodiscard]] double get_sample_time() const {return dt_;}

private:
    static MatrixXd compute_A_t(double omega, double t);
    static MatrixXd compute_b_t(double omega, double t);

    double mpc_qp(
        const Vector2d& Xcom_0, double p_0,
        double xcom_dot_des
    );

    void compute_desired_footholds(const Vector2d& p_star);

    std::tuple<VectorXd, VectorXd, VectorXd> compute_swing_feet_trajectories(
        const Vector3d& plane_coeffs,
        const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities
    );

    void switch_swing_feet(const std::vector<Vector3d>& feet_positions);

    std::tuple<Vector3d, Vector3d, Vector3d> get_des_base_pose(
        const Vector2d& Xcom_0, const Vector2d& Ycom_0,
        double px_0, double py_0
    );

    generalized_pose::GeneralizedPoseStruct mpc(
        const Vector3d& p_com, const Vector3d& v_com, const Vector3d& a_com,
        const Vector2d& vel_cmd, double yaw_rate_cmd,
        const Vector3d& plane_coeffs,
        const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities
    );

    /// @brief Return true when the robot should stop moving.
    /// @details True when the commanded linear and angular velocity has been equal to 0 for a number of robot steps grater than _max_fixed_steps and the previous swing phase has finished.
    bool check_stop(const Vector2d& vel_cmd, double yaw_rate_cmd);

    void correct_with_terrain_plane(
        const Vector3d& plane_coeffs,
        generalized_pose::GeneralizedPoseStruct& gen_pose
    ) const;

    bool stop_flag_ = true;

    /// @brief Number of future steps over which the optimization is performed
    int n_ = 3;

    // Cost function matrices
    // cost = sum 1/2 [ (xcom_dot - xcom_dot_des)^T Q (xcom_dot - xcom_dot_des) + (px - px_m)^T R (px - px_m) ]
    MatrixXd Q_ = 1000 * MatrixXd::Identity(n_, n_);
    MatrixXd R_ = 1 * MatrixXd::Identity(n_, n_);

    /// @brief Planner time step
    double dt_ = 1. / 200.;

    /// @brief Time normalized stride phase
    double phi_ = 0.;

    /// @brief Swing feet names
    std::vector<std::string> swing_feet_names_ = {"LF", "RH"};

    /// @brief Kinematic reachability limit for the stance feet
    double step_reachability_ = 0.2;

    // Parameters used to describe the feet position with respect to the ZMP
    double theta_0_ = 0.64;
    double r_ = 0.5;

    /// @brief Commanded yaw angle
    double dtheta_ = 0.;

    Interpolator interpolator_ = Interpolator(
        InterpolationMethod::Spline_3rd, 0.2,
        0.1, 0.0,
        -0.025
    );

    /// @brief Initial swing feet positions at the start of the swing phase. It is the last contact position.
    std::vector<Vector3d> init_pos_swing_feet_ = {};

    /// @brief Desired foothold positions of the swing feet at the end of the swing phase.
    std::vector<Vector2d> final_pos_swing_feet_ = {};

    /// @brief Desired height of the center of mass
    double height_com_ = 0.5;

    // The planner stops moving the feet when the reference velocity (linear and angular) has been equal to zero for a number of steps equal to max_fixed_steps.
    int max_fixed_steps_ = 6;
    int fixed_steps_ = 6;

    /// @brief Short ordered names of the swing feet.
    std::vector<std::string> all_feet_names_ = {"LF", "RF", "LH", "RH"};

    /// @brief Number of samples of a swing trajectory computed by compute_trajectory_sample_points().
    int n_sample_points_ = 10;

    /// @brief
    bool interpolate_swing_feet_from_current_position_ = false;
};

}