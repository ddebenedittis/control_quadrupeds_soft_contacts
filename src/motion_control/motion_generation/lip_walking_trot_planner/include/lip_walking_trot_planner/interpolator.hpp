#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iostream>



namespace lip_walking_trot_planner
{

/* ========================================================================== */
/*                                 DECLARATION                                */
/* ========================================================================== */

using namespace Eigen;


enum class InterpolationMethod {
    Spline_5th,
    Spline_3rd,
    Cycloid,
    Last
};


/// @brief Interpolate the foot position from an initial position and a desired foothold position.
class Interpolator {
public:
    Interpolator() = default;

    Interpolator(
        InterpolationMethod method, double step_duration,
        double step_height, double step_horizontal_phase_delay,
        double foot_penetration
    );

    /// @brief Compute the positions, velocities, and accelerations that interpolate the foot position from the initial position to the desired foothold position with null starting and ending velocities.
    /// @param init_pos Initial foot position
    /// @param end_pos Desired foothold position
    /// @param phi Swing phase \in [0; 1]
    /// @return {position, velocity, acceleration} of the swing foot
    [[nodiscard]] std::tuple<Vector3d, Vector3d, Vector3d> interpolate(
        const Vector3d& init_pos, const Vector3d& end_pos, double phi
    ) const;

    /// @brief Compute the positions, velocities, and accelerations that interpolate the foot position from the initial position to the desired foothold position with null starting and ending velocities.
    /// @param init_pos initial foot position
    /// @param init_vel initial foot velocity
    /// @param end_pos desired foot position
    /// @param end_vel desired foot velocity
    /// @param phi swing phase (\in [0, 1])
    /// @param dt sampling time
    /// @param step_duration time duration of the swing phase of a foot
    /// @return {position, velocity, acceleration} of the swing foot
    [[nodiscard]] std::tuple<Vector3d, Vector3d, Vector3d> interpolate(
        const Vector3d& init_pos, const Vector3d& init_vel,
        const Vector3d& end_pos, const Vector3d& end_vel,
        double phi, double dt
    ) const;

    /// @brief Compute the position, velocity, and acceleration of a point that interpolates between two points with null starting and ending velocities using a polynomial spline.
    /// 
    /// @param p_i initial position
    /// @param p_f final position
    /// @param phi phase (\in [0; 1])
    /// @param method spline type
    /// @return {position, velocity, acceleration}
    template <typename T>
    static std::tuple<T, T, T> spline(
        const T& p_i, const T& p_f, double phi, InterpolationMethod method
    );

    /// @brief Compute the position, velocity, and acceleration of a point that interpolates between two points with non-null starting and ending velocities using a polynomial spline.
    /// 
    /// @tparam T 
    /// @param p_i initial position
    /// @param v_i initial velocity
    /// @param p_f final position
    /// @param v_f final velocity
    /// @param phi phase
    /// @param method spline type
    /// @return {position, velocity, acceleration}
    static std::tuple<Vector2d, Vector2d, Vector2d> spline(
        const Vector2d& p_i, const Vector2d& v_i,
        const Vector2d& p_f, const Vector2d& v_f,
        double phi, InterpolationMethod method
    );

    /// @brief Compute the position, velocity, and acceleration of a point that interpolates between two points with non-null starting and ending velocities using a polynomial spline.
    /// 
    /// @param p_i initial position
    /// @param v_i initial velocity
    /// @param p_f final position
    /// @param v_f final velocity
    /// @param phi phase (\in [0, 1])
    /// @param method spline type
    /// @return {position, velocity, acceleration}
    static std::tuple<double, double, double> spline(
        double p_i, double v_i,
        double p_f, double v_f,
        double phi, InterpolationMethod method
    );

    /* =============================== Setters ============================== */

    void set_method(InterpolationMethod method) {this->method_ = method;}

    int set_step_duration(double step_duration)
    {
        if (step_duration > 0) {
            this->step_duration_ = step_duration;
        } else {
            std::cerr << "The step duration must be a positive value." << std::endl;
            return 1;
        }

        return 0;
    }

    int set_step_horizontal_phase_delay(double horizontal_phase_delay)
    {
        if (horizontal_phase_delay >= 0 && horizontal_phase_delay < 1) {
            this->step_horizontal_phase_delay_ = horizontal_phase_delay;
        } else {
            std::cerr << "The horizontal phase delay must be in >= 0 and < 1." << std::endl;
            return 1;
        }

        return 0;
    }

    int set_step_height(double step_height)
    {
        if (step_height > 0) {
            this->step_height_ = step_height;
        } else {
            std::cerr << "The step height must be > 0." << std::endl;
            return 1;
        }

        return 0;
    }

    void set_foot_penetration(double foot_penetration) {this->foot_penetration_ = foot_penetration;}

    /* =============================== Getters ============================== */

    [[nodiscard]] double get_step_duration() const {return step_duration_;}

    [[nodiscard]] double get_step_height() const {return step_height_;}

    [[nodiscard]] double get_foot_penetration() const {return foot_penetration_;}

private:
    /// @brief Compute a modified phi that is 0 when the phase is smaller that _horizontal_phase_delay/2 or bigger than (1 - _horizontal_phase_delay/2). This modified phi is used to ensure that the foot horizontal velocity is zero when it leaves and touches the ground.
    [[nodiscard]] double _compute_modified_phi(double phi) const
    {
        double phi_m = (phi - this->step_horizontal_phase_delay_/2) / (1 - this->step_horizontal_phase_delay_);

        return std::min(std::max(0., phi_m), 1.);
    }

    /// @brief Compute the interpolation between two quantities using a polynomial spline.
    ///
    /// @tparam T
    /// @param p_i initial position
    /// @param p_f final position
    /// @param phi swing phase (\in [0; 1])
    /// @return std::tuple<T, T, T> {pos, vel, acc}
    template <typename T>
    std::tuple<T, T, T> spline(
        const T& p_i, const T& p_f, double phi
    ) const {
        return spline(
            p_i, p_f, phi, method_
        );
    }

    /// @brief Compute and return the interpolated position, velocity and acceleration of a swing foot using a polynomial spline.
    /// @param init_pos initial position of the swing foot
    /// @param end_pos desired foothold position of the swing foot
    /// @param phi swing phase
    /// @return {position, velocity, acceleration} of the swing foot
    [[nodiscard]] std::tuple<Vector3d, Vector3d, Vector3d> foot_trajectory_spline(
        const Vector3d& init_pos, const Vector3d& end_pos, double phi
    ) const;

    [[nodiscard]] std::tuple<Vector3d, Vector3d, Vector3d> foot_trajectory_spline(
        const Vector3d& init_pos, const Vector3d& init_vel,
        const Vector3d& end_pos, const Vector3d& end_vel,
        double phi, double dt
    ) const;

    /// @brief Compute and return the interpolated position, velocity and acceleration of a swing foot using a cycloid.
    /// @param init_pos initial position of the swing foot
    /// @param d sted length
    /// @param theta yaw angle of the robot
    /// @param phi swing phase
    /// @return {position, velocity, acceleration} of the swing foot
    [[nodiscard]] std::tuple<Vector3d, Vector3d, Vector3d> foot_trajectory_cycloid(
        const Vector3d& init_pos, double d, double theta, double phi
    ) const;

    void limit_feet_vel_acc(Vector3d& feet_velocities, Vector3d& feet_accelerations) const;

    /* ====================================================================== */

    InterpolationMethod method_ = InterpolationMethod::Spline_3rd;

    // Time duration of a step swing (both upwards and downwards movement).
    double step_duration_ = 0.2;

    // Phase delay before which the foot starts moving in the horizontal direction.
    double step_horizontal_phase_delay_ = 0.1;

    // How much the foot is raised during the swing phase.
    double step_height_ = 0.1;

    // Positive when the foot penetrates the terrain.
    double foot_penetration_ = - 0.025;

    double max_foot_velocity = 4;
    double max_foot_acceleration = 20;
};

}
