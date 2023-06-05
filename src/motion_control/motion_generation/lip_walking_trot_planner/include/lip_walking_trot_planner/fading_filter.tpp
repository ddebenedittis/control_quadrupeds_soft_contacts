#pragma once

#include <cmath>
#include <iostream>


namespace lip_walking_trot_planner {



/* ========================================================================== */
/*                                 DECLARATION                                */
/* ========================================================================== */

enum class FilterOrder {
    First,
    Second
};


/// @brief Implement a simple fading filter of first or second order (a low-pass filter )
template <typename T>
class FadingFilter {
public:
    FadingFilter() = default;

    FadingFilter(T x)
    : x_(x),
      x_dot_(x),
      initialized_(true)
    {
        for (auto& elem : x_dot_) {
            elem = 0;
        }
    }

    /// @brief Filter the measurement and return the filtered value.
    /// 
    /// @param meas measurement
    /// @param Ts Sample time
    /// @return Filtered measurement
    T filter(const T& meas, double Ts = 0);

    /* =============================== Setters ============================== */
    
    void set_order(FilterOrder order) {order_ = order;}

    int set_order(int order)
    {
        switch (order) {
        case 1:
            order_ = FilterOrder::First;
            return 0;
        case 2:
            order_ = FilterOrder::Second;
            return 0;
        default:
            std::cerr << "In lip_walking_trot_planner::FadingFilter.set_order(int), the input order is not an acceptable value. It bust be in {1, 2}." << std::endl;
            return 1;
        }
    }

    int set_beta(double beta)
    {
        if (beta < 0 || beta > 1) {
            std::cerr << "In lip_walking_trot_planner::FadingFilter.set_beta(double), the beta coefficient must be in [0; 1]." << std::endl;
            return 1;
        } else {
            beta_ = beta;
            return 0;
        }
    }

private:
    // When false, initialize x_ to the meas and x_dot_ to 0. Otherwise, filter the measurement.
    bool initialized_ = false;

    FilterOrder order_ = FilterOrder::First;
    
    /// @brief beta_ \in [0, 1]. When beta_ is near 1, the estimate rapidly tracks the measurements. When beta_ is near 0, the estimate adapts to changes of the measurements more slowly.
    double beta_ = 0.9;
    
    T x_;
    T x_dot_;
};



/* ========================================================================== */
/*                                 DEFINITION                                 */
/* ========================================================================== */

template <typename T>
T FadingFilter<T>::filter(const T& meas, double Ts)
{
    switch (order_) {
    case FilterOrder::First:
        if (initialized_) {
            x_ = beta_ * x_ + (1 - beta_) * meas;
        } else {
            // Initialize the state with the current measurement.
            x_ = meas;

            initialized_ = true;
        }

        break;

    /* ====================================================================== */

    case FilterOrder::Second:
        if (initialized_) {
            double G = 1 - std::pow(beta_, 2);
            double H = std::pow((1 - beta_), 2);

            x_ += x_dot_ * Ts + G * (meas - (x_ + x_dot_ * Ts));
            x_dot_ += H / Ts * (meas - (x_ + x_dot_ * Ts));
        } else {
            // Initialize the state with the current measurement and the state derivative with zero.
            x_ = meas;
            x_dot_ = meas;
            for (auto& elem : x_dot_) {
                elem = 0;
            }

            initialized_ = true;
        }

        break;
    }

    return x_;
}

}
