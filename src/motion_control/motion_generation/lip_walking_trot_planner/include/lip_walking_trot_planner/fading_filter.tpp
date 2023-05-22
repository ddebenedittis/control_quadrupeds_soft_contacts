#pragma once

#include <cmath>
#include <iostream>


namespace lip_walking_trot_planner {

enum class FilterOrder {
    First,
    Second
};


template <typename T>
class FadingFilter {
public:
    FadingFilter() = default;

    FadingFilter(T x)
    : x_(x),
      x_dot_(x),
      initialized_(true)
    {
        for (auto elem : x_dot_) {
            elem = 0;
        }
    }

    T filter(const T& meas, double Ts = 0);
    
    void set_order(FilterOrder order) {order_ = order;}

    void set_beta(double beta)
    {
        if (beta < 0 || beta > 0) {
            std::cerr << "The fading filter beta coefficient must be in [0; 1]" << std::endl;
        } else {
            beta_ = beta;
        }
    }

private:
    bool initialized_ = false;

    FilterOrder order_ = FilterOrder::First;
    
    /// @brief With beta_ = 1 the estimate is equal to the measurement.
    double beta_ = 0.9;
    
    T x_;
    T x_dot_;
};


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
