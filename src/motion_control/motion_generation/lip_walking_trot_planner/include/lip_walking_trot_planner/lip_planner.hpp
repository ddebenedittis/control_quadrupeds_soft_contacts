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
        const Ref<Vector3d>& init_pos = (Vector3d() << 0, 0, 0).finished(),
        double init_yaw = 0,
        const std::vector<Vector3d>& feet_positions = {}
    );

    /// @brief Compute the desired generalized pose.
    /// 
    /// @param pos_com Position of the center of mass
    /// @param vel_com Velocity of the center of mass
    /// @param acc_com Acceleration of the center of mass
    /// @param vel_cmd Linear velocity command {vel_forward, vel_lateral}
    /// @param yaw_rate_cmd Yaw rate command
    /// @param plane_coeffs Terrain plane coefficients {a, b, c} where the plane is z = a x + b y + c
    /// @param feet_positions 
    /// @param feet_velocities 
    /// @param time
    /// @return generalized_pose::GeneralizedPoseStruct 
    std::vector<generalized_pose::GeneralizedPoseStruct> update(
        const Ref<Vector3d>& pos_com, const Ref<Vector3d>& vel_com, const Ref<Vector3d>& acc_com,
        const Ref<Vector2d>& vel_cmd, double yaw_rate_cmd,
        const Vector3d& plane_coeffs = {0, 0, 0},
        const std::vector<Vector3d>& feet_positions = {}, const std::vector<Vector3d>& feet_velocities = {},
        double time = 0
    );

    /// @brief Interpolate a position between p_i and p_f using a polynomial spline.
    /// 
    /// @param init_pos Initial position
    /// @param final_pos Final position
    /// @param phi Phase \in [0, 1]
    /// @param method Interpolation method
    /// @return {position, velocity, acceleration} 
    static std::tuple<Vector3d, Vector3d, Vector3d> spline(
        const Vector3d& init_pos, const Vector3d& final_pos, double phi, InterpolationMethod method
    ) {
        return Interpolator::spline<Vector3d>(init_pos, final_pos, phi, method);
    };

    /// @brief Return a list of trajectories of the swing feet sampled in n_sample_points_.
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

    int set_step_reachability(double step_reachability)
    {
        if (step_reachability <= 0) {
            std::cerr << "In MotionPlanner, the step reachability must be > 0." << std::endl;
            return 1;
        }
        this->step_reachability_ = step_reachability;
        return 0;
    }

    void set_feet_theta(double feet_theta)
    {
        this->theta_0_ = feet_theta;
        update_initial_conditions();
    }

    int set_feet_r(double feet_r)
    {
        if (feet_r <= 0) {
            std::cerr << "In MotionPlanner, the feet r must be > 0." << std::endl;
            return 1;
        }

        this->r_ = feet_r;
        update_initial_conditions();
        return 0;
    }

    int set_base_height(double base_height)
    {
        if (base_height <= 0) {
            std::cerr << "In MotionPlanner, the base height must be > 0." << std::endl;
            return 1;
        }

        this->height_com_ = base_height;
        return 0;
    }

    void set_max_fixed_steps(int max_fixed_steps)
    {
        this->max_fixed_steps_ = max_fixed_steps;
        this->fixed_steps_ = max_fixed_steps;
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

    int set_dt_gen_poses(double dt)
    {
        if (dt > 0) {
            dt_gen_poses_ = dt;
            return 0;
        }

        std::cerr << "In lip_walking_trot_planner::MotionPlanner.set_dt_gen_poses(double) the sample time of the generalized poses must be positive." << std::endl;
        return 1;
    }

    int set_n_gen_poses(int n)
    {
        if (n > 0) {
            n_gen_poses_ = n;
            return 0;
        }

        std::cerr << "In lip_walking_trot_planner::MotionPlanner.set_n_gen_poses(int) the number of gen_poses computed must be positive." << std::endl;
        return 1;
    }

    /* =============================== Getters ============================== */

    [[nodiscard]] double get_dtheta() const {return yaw_;}

    [[nodiscard]] double get_height_com() const {return height_com_;}

    [[nodiscard]] double get_sample_time() const {return dt_;}

    /* ====================================================================== */

    [[nodiscard]] std::vector<std::string> get_other_feet(std::vector<std::string> feet_names) const
    {
        std::vector<std::string> new_feet_names = {};
        new_feet_names.reserve(all_feet_names_.size() - feet_names.size());

        for (const auto& foot_name : all_feet_names_) {
            if (std::find(feet_names.begin(), feet_names.end(), foot_name) == feet_names.end()) {
                new_feet_names.push_back(foot_name);
            }
        }

        return new_feet_names;
    }

private:
    [[nodiscard]] std::vector<generalized_pose::GeneralizedPoseStruct> stand_still(
        const Vector3d& plane_coeffs
    ) const;

    std::vector<generalized_pose::GeneralizedPoseStruct> mpc(
        const Vector3d& pos_com, const Vector3d& vel_com, const Vector3d& acc_com,
        const Vector2d& vel_cmd, double yaw_rate_cmd,
        const Vector3d& plane_coeffs,
        const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities,
        double time
    );

    [[nodiscard]] std::vector<Vector2d> downsampled_solve_qps(
        const Vector2d& X_com_0, const Vector2d& Y_com_0,
        double x_zmp_0, double y_zmp_0,
        const Vector2d& vel_cmd, double yaw_rate_cmd
    );

    [[nodiscard]] std::vector<Vector2d> solve_qps(
        const Vector2d& X_com_0, const Vector2d& Y_com_0,
        double x_zmp_0, double y_zmp_0,
        const Vector2d& vel_cmd, double yaw_rate_cmd
    ) const;

    /// @brief Formulate and solve the QP problem along a single direction.
    /// 
    /// @param Xcom_0 State along a single direction {x_com, x_dot_com}
    /// @param x_zmp_0 Measured zmp coordinate at the current step
    /// @param xcom_dot_des Desired velocity along a single direction
    /// @return Optimal ZMP coordinate at the next step
    [[nodiscard]] std::vector<double> mpc_qp(
        const Vector2d& X_com_0, double x_zmp_0,
        double x_com_dot_des
    ) const;

    static Matrix2d compute_A_t(double omega, double time);
    static Vector2d compute_b_t(double omega, double time);

    std::vector<generalized_pose::GeneralizedPoseStruct> compute_gen_poses(
        const Vector2d& X_com_0, const Vector2d& Y_com_0,
        double x_zmp_0, double y_zmp_0,
        std::vector<Vector2d>& pos_zmp_star,
        double yaw_rate_cmd,
        const Vector3d& plane_coeffs,
        const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities
    );

    void compute_desired_footholds(const std::vector<Vector2d>& pos_zmp);

    std::tuple<std::vector<VectorXd>, std::vector<VectorXd>, std::vector<VectorXd>> compute_swing_feet_trajectories(
        const Vector3d& plane_coeffs,
        const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities
    );

    void switch_swing_feet(const std::vector<Vector3d>& feet_positions);

    [[nodiscard]] std::tuple<std::vector<Vector3d>, std::vector<Vector3d>, std::vector<Vector3d>> get_des_base_poses(
        const Vector2d& X_com_0, const Vector2d& Y_com_0,
        const std::vector<Vector2d>& pos_zmp
    ) const;

    /// @brief Return true when the robot should stop moving.
    /// @details True when the commanded linear and angular velocity has been equal to 0 for a number of robot steps grater than _max_fixed_steps and the previous swing phase has finished.
    bool check_stop(const Vector2d& vel_cmd, double yaw_rate_cmd);

    static void correct_base_pose_with_terrain_plane(
        const Vector3d& plane_coeffs,
        double yaw,
        generalized_pose::GeneralizedPoseStruct& gen_pose
    );

    /* ====================================================================== */

    /// @brief When true, the robot stands still and does not solve the mpc.
    bool stop_flag_ = true;

    /// @brief Number of future steps over which the optimization is performed.
    int n_steps_prediction_horizon = 3;

    // Cost function matrices
    // cost = sum 1/2 [ (xcom_dot - xcom_dot_des)^T Q (xcom_dot - xcom_dot_des) + (px - px_m)^T R (px - px_m) ]
    MatrixXd Q_ = 1000 * MatrixXd::Identity(n_steps_prediction_horizon, n_steps_prediction_horizon);
    MatrixXd R_ = 1 * MatrixXd::Identity(n_steps_prediction_horizon, n_steps_prediction_horizon);

    /// @brief Planner time step
    double dt_ = 1. / 200.;

    /// @brief Time normalized stride phase
    double phi_ = 0.;

    /// @brief Swing feet names
    std::vector<std::string> swing_feet_names_ = {"LF", "RH"};

    /// @brief Kinematic reachability limit for the stance feet
    double step_reachability_ = 0.2;

    // Parameters used to describe the feet position with respect to the ZMP.
    // The LF foot is positioned in r_ * [cos(theta_0_), sin(theta_0_)] with respect to the base in base frame.
    double theta_0_ = 0.64;
    double r_ = 0.5;

    /// @brief Commanded yaw angle
    double yaw_ = 0.;

    Interpolator interpolator_ = Interpolator(
        InterpolationMethod::Spline_3rd,
        /* step_duration*/ 0.2,
        /* step_height */ 0.1,
        /* horizontal_phase_delay */ 0.0,
        /* foot_penetration */ -0.025
    );

    /// @brief Initial swing feet positions at the start of the swing phase. It is the last contact position.
    std::vector<Vector3d> init_pos_swing_feet_ = {};

    /// @brief Desired foothold positions of the swing feet at the end of the swing phase.
    std::vector<std::vector<Vector2d>> final_pos_swing_feet_ = {};

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

    /// @brief Sample time at which the output generalized poses are sampled.
    double dt_gen_poses_ = 1. / 200.;
    /// @brief Number of samples of output generalized poses.
    int n_gen_poses_ = 4;

    /// @brief 
    double last_time = 0;

    /// @brief Optimal ZMP positions computed.
    std::vector<Vector2d> pos_zmp_star_;

    // Used to downsample the optimal ZMP computation. Update the optimal ZMP position every once every downsample_factor_ times.
    /// @brief The ZMP position is only updated when counter_ == 0.
    int counter_ = 0;
    /// @brief counter_ is reset to 0 after it becomes == downsample_factor_.
    int downsample_factor_ = 1;
};

} // lip_walking_trot_planner