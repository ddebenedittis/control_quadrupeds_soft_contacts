#pragma once

#include <array>
#include <math.h>
#include <stdio.h>



// Quaternion multiplication function
std::array<double, 4> quat_mult(std::array<double, 4> q, std::array<double, 4> p)
{
    // Declare the output array
    std::array<double, 4> r;
    
    // Perform the quaternion multiplication
    r[0] =    q[0] * p[3] + q[1] * p[2] - q[2] * p[1] + q[3] * p[0];
    r[1] =  - q[0] * p[2] + q[1] * p[3] + q[2] * p[0] + q[3] * p[1];
    r[2] =    q[0] * p[1] - q[1] * p[0] + q[2] * p[3] + q[3] * p[2];
    r[3] =  - q[0] * p[0] - q[1] * p[1] - q[2] * p[2] + q[3] * p[3];

    // Return the array directly
    return r;
}

// Quaternion exponential of a 3D vector
std::array<double, 4> quat_exp_vec(std::array<double, 3> v)
{
    std::array<double, 4> q;
    double norm_v;

    // Compute the norm of v
    norm_v = 0.0;

    for(int i=0; i<3; i++)
        norm_v += v[i] * v[i];

    norm_v = sqrt (norm_v);

    // Compute the quaternion q
    if(norm_v == 0.0 ) {
        q[0] = q[1] = q[2] = 0.0;
    }
    else {
        for(int i=0; i<3; i++)
            q[i] = v[i] / norm_v * sin(norm_v);
    }
        
    q[3] = cos (norm_v);

    return q;
}

double quat_norm(std::array<double, 4>& q)
{
    double norm = 0;

    for(int i=0; i<4; i++)
        norm += q[i] * q[i];

    return sqrt(norm);
}

// Normalize a quaternion
void normalize_quat(std::array<double, 4>& q)
{
    double norm = 0;

    for(int i=0; i<4; i++)
        norm += q[i] * q[i];

    norm = sqrt (norm);

    for(int i=0; i<4; i++)
        q[i] = q[i] / norm;
}

// Integrate a quaternion with its body frame angular velocity
std::array<double, 4> quat_int(std::array<double, 4> q, std::array<double, 3> w_b, double dt)
{
    std::array<double, 4> q_out;

    for (int i=0; i<3; i++)
        // w_b[i] = 1 / 2 * dt * w_b[i]; does not work. Why?
        w_b[i] = 0.5 * dt * w_b[i];

    q_out = quat_mult(q, quat_exp_vec(w_b));

    normalize_quat(q_out);

    return q_out;
}