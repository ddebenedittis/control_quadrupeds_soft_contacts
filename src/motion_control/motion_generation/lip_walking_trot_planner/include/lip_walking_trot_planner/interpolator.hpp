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
        double horizontal_phase_delay, double step_height,
        double foot_penetration
    );

    /// @brief Interpolate the foot position from the initial position to the desired foothold position.
    /// @param init_pos Initial foot position
    /// @param end_pos Desired foothold position
    /// @param phi Swing phase \in [0; 1]
    /// @return {position, velocity, acceleration} of the swing foot
    std::tuple<Vector3d, Vector3d, Vector3d> interpolate(
        const Vector3d& init_pos, const Vector3d& end_pos, double phi
    );

    /// @brief Interpolate the position between two points using a polynomial spline (as specified in the _method variable).
    template <typename T>
    static std::tuple<T, T, T> spline(
        const T& p_i, const T& p_f, double phi, InterpolationMethod method
    );

    /* =============================== Setters ============================== */

    void set_method(InterpolationMethod method) {this->method_ = method;}

    void set_step_duration(double step_duration)
    {
        if (step_duration > 0) {
            this->step_duration_ = step_duration;
        } else {
            std::cerr << "The step duration must be a positive value." << std::endl;
        }
    }

    void set_horizontal_phase_delay(double horizontal_delay)
    {
        if (horizontal_delay >= 0 && horizontal_delay <= 1) {
            this->horizontal_phase_delay_ = horizontal_delay;
        } else {
            std::cerr << "The horizontal phase delay must be in [0; 1]." << std::endl;
        }
    }

    void set_step_height(double step_height)
    {
        if (step_height > 0) {
            this->step_height_ = step_height;
        } else {
            std::cerr << "The step height must be > 0." << std::endl;
        }
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
        double phi_m = (phi - this->horizontal_phase_delay_/2) / (1 - this->horizontal_phase_delay_);

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
    ) {
        return spline(
            p_i, p_f, phi, method_
        );
    }

    /// @brief Compute and return the interpolated position, velocity and acceleration of a swing foot using a polynomial spline.
    /// @param init_pos initial position of the swing foot
    /// @param end_pos desired foothold position of the swing foot
    /// @param phi swing phase
    /// @return {position, velocity, acceleration} of the swing foot
    std::tuple<Vector3d, Vector3d, Vector3d> foot_trajectory_spline(
        const Vector3d& init_pos, const Vector3d& end_pos, double phi
    );

    /// @brief Compute and return the interpolated position, velocity and acceleration of a swing foot using a cycloid.
    /// @param init_pos initial position of the swing foot
    /// @param d sted length
    /// @param theta yaw angle of the robot
    /// @param phi swing phase
    /// @return {position, velocity, acceleration} of the swing foot
    std::tuple<Vector3d, Vector3d, Vector3d> foot_trajectory_cycloid(
        const Vector3d& init_pos, double d, double theta, double phi
    );

    /* ====================================================================== */

    InterpolationMethod method_ = InterpolationMethod::Spline_3rd;

    // Time duration of a step swing (both upwards and downwards movement).
    double step_duration_ = 0.2;

    // Phase delay before which the foot starts moving in the horizontal direction.
    double horizontal_phase_delay_ = 0.1;

    // How much the foot is raised during the swing phase.
    double step_height_ = 0.1;

    // Positive when the foot penetrates the terrain.
    double foot_penetration_ = - 0.025;
};

}
