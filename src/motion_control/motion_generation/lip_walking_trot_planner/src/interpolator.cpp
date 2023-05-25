#include "lip_walking_trot_planner/interpolator.hpp"


namespace lip_walking_trot_planner {


/* =============================== Constructor ============================== */

Interpolator::Interpolator(
    InterpolationMethod method, double step_duration,
    double step_height, double step_horizontal_phase_delay,
    double foot_penetration
): method_(method)
{
    set_step_duration(step_duration);
    set_step_height(step_height);
    set_step_horizontal_phase_delay(step_horizontal_phase_delay);
    set_foot_penetration(foot_penetration);
}


/* =============================== Interpolate ============================== */

std::tuple<Vector3d, Vector3d, Vector3d> Interpolator::interpolate(
    const Vector3d& init_pos, const Vector3d& end_pos, double phi
) const {
    if (this->method_ == InterpolationMethod::Spline_3rd ||
        this->method_ == InterpolationMethod::Spline_5th) {
        return foot_trajectory_spline(init_pos, end_pos, phi);
    } else if (this->method_ == InterpolationMethod::Cycloid) {
        double step_length = (end_pos - init_pos).norm();
        double theta = std::atan2(
            end_pos[1] - init_pos[1],
            end_pos[0] - init_pos[0]
        );

        return foot_trajectory_cycloid(init_pos, step_length, theta, phi);
    }
    else {
        // This case should never be reached.
        return foot_trajectory_spline(init_pos, end_pos, phi);
    }
}


/* =============================== Interpolate ============================== */

std::tuple<Vector3d, Vector3d, Vector3d> Interpolator::interpolate(
    const Vector3d& init_pos, const Vector3d& init_vel,
    const Vector3d& end_pos, const Vector3d& end_vel,
    double phi, double dt
) const {
    if (this->method_ == InterpolationMethod::Spline_3rd ||
        this->method_ == InterpolationMethod::Spline_5th) {
        return foot_trajectory_spline(
            init_pos, init_vel,
            end_pos, end_vel,
            phi, dt
        );
    } else {
        // This case should never be reached.
        return foot_trajectory_spline(
            init_pos, init_vel,
            end_pos, end_vel,
            phi, dt
        );
    }
}


/* ================================= Spline ================================= */

template <typename T>
std::tuple<T, T, T> Interpolator::spline(
    const T& p_i, const T& p_f, double phi, InterpolationMethod method
) {
    // phi /in [0, 1]

    double f_t = 0;
    double f_t_dot = 0;
    double f_t_ddot = 0;

    // Local time of transition phase and its derivatives
    if (method == InterpolationMethod::Spline_5th) {
        f_t = std::pow(phi, 3) * (10 - 15 * phi + 6 * std::pow(phi, 2));
        f_t_dot = 30 * std::pow(phi, 2) - 60 * std::pow(phi, 3) + 30 * std::pow(phi, 4);
        f_t_ddot = 60 * phi - 180 * std::pow(phi, 2) + 120 * std::pow(phi, 3);

    } else if (method == InterpolationMethod::Spline_3rd) {
        f_t = std::pow(phi, 2) * (3 - 2*phi);
        f_t_dot = 6*phi - 6*std::pow(phi, 2);
        f_t_ddot = 6 - 12*phi;
    }


    // Polynomial spline and its derivatives
    T p_t = (1 - f_t) * p_i + f_t * p_f;
    T v_t = - f_t_dot * p_i + f_t_dot * p_f;
    T a_t = - f_t_ddot * p_i + f_t_ddot * p_f;

    return {p_t, v_t, a_t};
}

template std::tuple<double, double, double>
Interpolator::spline<double>(const double&, const double&, double, InterpolationMethod);

template std::tuple<Vector2d, Vector2d, Vector2d>
Interpolator::spline<Vector2d>(const Vector2d&, const Vector2d&, double, InterpolationMethod);

template std::tuple<Vector3d, Vector3d, Vector3d>
Interpolator::spline<Vector3d>(const Vector3d&, const Vector3d&, double, InterpolationMethod);


/* ================================= Spline ================================= */

std::tuple<Vector2d, Vector2d, Vector2d> Interpolator::spline(
    const Vector2d& p_i, const Vector2d& v_i,
    const Vector2d& p_f, const Vector2d& v_f,
    double phi, InterpolationMethod method
) {
    // phi /in [0, 1]

    Array2d f_t = Vector2d::Zero();
    Array2d f_t_dot = Vector2d::Zero();
    Array2d f_t_ddot = Vector2d::Zero();

    // Local time of transition phase and its derivatives
    if (method == InterpolationMethod::Spline_3rd) {
        Array2d b = v_i;
        Array2d c = 3 - v_f.array() - 2*v_i.array();
        Array2d d = v_i.array() + v_f.array() - 2;

        f_t = b * phi + c * std::pow(phi, 2) + d * std::pow(phi, 3);
        f_t_dot = b + 2*c * phi + 3*d * std::pow(phi, 2);
        f_t_ddot = 2*c + 6*d * phi;
    }


    // Polynomial spline and its derivatives
    Vector2d p_t = (1 - f_t) * p_i.array() + f_t * p_f.array();
    Vector2d v_t = - f_t_dot * p_i.array() + f_t_dot * p_f.array();
    Vector2d a_t = - f_t_ddot * p_i.array() + f_t_ddot * p_f.array();

    return {p_t, v_t, a_t};
}


/* ================================= Spline ================================= */

std::tuple<double, double, double> Interpolator::spline(
    double p_i, double v_i,
    double p_f, double v_f,
    double phi, InterpolationMethod method
) {
    // phi /in [0, 1]

    double f_t = 0;
    double f_t_dot = 0;
    double f_t_ddot = 0;

    // Local time of transition phase and its derivatives
    if (method == InterpolationMethod::Spline_3rd) {
        double b = v_i;
        double c = 3 - v_f - 2*v_i;
        double d = v_i + v_f - 2;

        f_t = b * phi + c * std::pow(phi, 2) + d * std::pow(phi, 3);
        f_t_dot = b + 2*c * phi + 3*d * std::pow(phi, 2);
        f_t_ddot = 2*c + 6*d * phi;
    }


    // Polynomial spline and its derivatives
    double p_t = (1 - f_t) * p_i + f_t * p_f;
    double v_t = - f_t_dot * p_i + f_t_dot * p_f;
    double a_t = - f_t_ddot * p_i + f_t_ddot * p_f;

    return {p_t, v_t, a_t};
}


/* ========================= Foot_trajectory_spline ========================= */

std::tuple<Vector3d, Vector3d, Vector3d> Interpolator::foot_trajectory_spline(
    const Vector3d& init_pos, const Vector3d& end_pos, double phi
) const {
    double p_z, v_z, a_z;

    if (phi <= 0.5) {
        double phi_2 = phi * 2;
        std::tie(p_z, v_z, a_z) = spline(0., step_height_, phi_2);
    } else {
        double phi_2 = 2 * phi - 1;
        std::tie(p_z, v_z, a_z) = spline(step_height_, - foot_penetration_, phi_2);
    }

    p_z += init_pos[2];

    // The velocity and acceleration must be scaled to take into account the time in which the swing is performed (since phi is a normalized quantity).
    v_z /= step_duration_ / 2;
    a_z /= step_duration_ / 2;

    Vector2d init_pos_xy = init_pos.head(2);
    Vector2d end_pos_xy = end_pos.head(2);
    auto [p_xy, v_xy, a_xy] = spline(init_pos_xy, end_pos_xy, phi);
    v_xy /= step_duration_;
    a_xy /= step_duration_;

    Vector3d p_t = {p_xy[0], p_xy[1], p_z};
    Vector3d v_t = {v_xy[0], v_xy[1], v_z};
    Vector3d a_t = {a_xy[0], a_xy[1], a_z};

    return {p_t, v_t, a_t};
}


/* ========================= Foot_trajectory_spline ========================= */

std::tuple<Vector3d, Vector3d, Vector3d> Interpolator::foot_trajectory_spline(
    const Vector3d& init_pos, const Vector3d& init_vel,
    const Vector3d& end_pos, const Vector3d& end_vel,
    double phi, double dt
) const {
    double p_z, v_z, a_z;

    if (phi <= 0.5) {
        double remaining_upward_phase_duration = step_duration_ / 2 * (0.5 - phi);

        double phi_up = dt / remaining_upward_phase_duration;
        std::tie(p_z, v_z, a_z) = spline(
            0, init_vel[2],
            step_height_, 0,
            phi_up, InterpolationMethod::Spline_3rd);

        v_z /= remaining_upward_phase_duration;
        a_z /= remaining_upward_phase_duration;
    } else {
        double remaining_downward_phase_duration = step_duration_ / 2 * (1 - phi);

        double phi_down = dt / remaining_downward_phase_duration;
        std::tie(p_z, v_z, a_z) = spline(
            0, init_vel[2],
            - foot_penetration_, 0,
            phi_down, InterpolationMethod::Spline_3rd);

        v_z /= remaining_downward_phase_duration;
        a_z /= remaining_downward_phase_duration;
    }

    p_z += init_pos[2];

    Vector2d init_pos_xy = init_pos.head(2);
    Vector2d init_vel_xy = init_vel.head(2);
    Vector2d end_pos_xy = end_pos.head(2);
    Vector2d end_vel_xy = end_vel.head(2);

    double phi_hor = dt / step_duration_ / (1 - phi);
    auto [p_xy, v_xy, a_xy] = spline(
        init_pos_xy, init_vel_xy,
        end_pos_xy, end_vel_xy,
        phi_hor, InterpolationMethod::Spline_3rd);

    // The velocity and acceleration must be scaled to take into account the time in which the swing is performed (since phi is a normalized quantity).
    v_xy /= step_duration_ * (1 - phi);
    a_xy /= step_duration_ * (1 - phi);

    Vector3d p_t = {p_xy[0], p_xy[1], p_z};
    Vector3d v_t = {v_xy[0], v_xy[1], v_z};
    Vector3d a_t = {a_xy[0], a_xy[1], a_z};

    return {p_t, v_t, a_t};
}


/* ========================= Foot_trajectory_cycloid ======================== */

std::tuple<Vector3d, Vector3d, Vector3d> Interpolator::foot_trajectory_cycloid(
    const Vector3d& init_pos, double d, double theta, double phi
) const {
    double T = this->step_duration_;
    double h = this->step_height_;

    double t = phi * T;

    // Compute the horizontal displacement, velocity, and acceleration.
    double phi_m = _compute_modified_phi(phi);
    double t_m = phi_m * T;

    double x = d * (t_m / T - 1 / (2 * M_PI) * std::sin(2 * M_PI * t_m / T));
    double x_dot = d / T * (1 - std::cos(2 * M_PI * t_m / T));
    double x_ddot = 2 * M_PI * d / std::pow(T, 2) * std::sin(2 * M_PI * t_m / T);

    // Compute the vertical position, velocity and acceleration.
    double z = 2 * h * (t/T - 1 / (4*M_PI) * std::sin(4 * M_PI * t / T));
    double z_dot = 2 * h / T * (1 - std::cos(4 * M_PI * t / T));
    double z_ddot = 2 * h / std::pow(T, 2) * 4 * M_PI * std::sin(4 * M_PI * t / T);

    if (t > T/2){
        z = 2 * h - z;
        z_dot = - z_dot;
        z_ddot = - z_ddot;
    }

    z += init_pos[2];

    Vector3d p_t = init_pos + Vector3d{x * std::cos(theta), x * std::sin(theta), z};
    Vector3d v_t = Vector3d{x_dot * std::cos(theta), x_dot * std::sin(theta), z_dot};
    Vector3d a_t = Vector3d{x_ddot * std::cos(theta), x_ddot * std::sin(theta), z_ddot};

    return {p_t, v_t, a_t};
}

}
