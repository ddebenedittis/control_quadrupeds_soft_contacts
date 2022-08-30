#include "static_walk_planner/static_walk_planner.hpp"



namespace static_walk_planner {

double pow(double base, int exponent) {
    if (exponent == 1) {
        return base;
    }
    else {
        return base * pow(base, exponent - 1);
    }
}

StaticWalkPlanner::StaticWalkPlanner() {}

GeneralizedPose StaticWalkPlanner::plan()
{

}

void StaticWalkPlanner::spline(
    Eigen::VectorXd p_i, Eigen::VectorXd p_f, double t,
    Eigen::VectorXd p_t, Eigen::VectorXd v_t, Eigen::VectorXd a_t
) {
    using std::pow;

    // Local time of transition phase and its derivatives
    double f_t, f_t_dot, f_t_ddot;

    if (spline_order_ == 5) {
        f_t = pow(t, 3) * (10 - 15 * t + 6 * pow(t,2));

        f_t_dot = 30 * pow(t,2) - 60 * pow(t,3) + 30 * pow(t,4);

        f_t_ddot = 60 * t - 180 * pow(t,2) + 120 * pow(t,3);
    }
    else if (spline_order_ == 3) {
        f_t = pow(t,2) * (3 - 2 * t);

        f_t_dot = 6 * t - 6 * pow(t,2);

        f_t_ddot = 6 - 12 * t;
    }

    // Polynomial spline and its derivatives
    p_t = (1 - f_t) * p_i + f_t * p_f;

    v_t = - f_t_dot * p_i + f_t_dot * p_f;

    a_t = - f_t_ddot * p_i + f_t_ddot * p_f;
}

} // namespace static_walk_planner