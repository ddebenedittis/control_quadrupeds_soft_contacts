#include "static_walk_planner/static_walk_planner.hpp"

#include <algorithm>
#include <math.h>



namespace static_walk_planner {

using namespace Eigen;



/* ========================================================================== */
/*                              STATICWALKPLANNER                             */
/* ========================================================================== */

StaticWalkPlanner::StaticWalkPlanner() {}


/* ========================================================================== */
/*                                STEP_TEMPLATE                               */
/* ========================================================================== */

void StaticWalkPlanner::step_template(GeneralizedPose& gen_pose)
{

}



/* ========================================================================== */
/*                               STEP_RAISE_FOOT                              */
/* ========================================================================== */

void StaticWalkPlanner::step_raise_foot(GeneralizedPose& gen_pose)
{

}



/* ========================================================================== */
/*                                    STEP                                    */
/* ========================================================================== */

void StaticWalkPlanner::step(GeneralizedPose& gen_pose)
{
    /* =========================== Initialization` ========================== */

    // First part of the initialization
    if (phi_ < init_phase_ / 2) {
        step_initialization_0(gen_pose);
    } 
    // Second part of the initialization
    else if (phi_ < init_phase_) {
        step_initialization_1(gen_pose);
    }

    /* ============================== Planning ============================== */

    else {
        step_plan(gen_pose);
    }

    /* ====================================================================== */

    // Base angular quantities
    gen_pose.base_quat << 0, 0, 0, 1;
    gen_pose.base_angvel << 0, 0, 0;

    /* ======================= Increase The Gait Phase ====================== */

    // Increase the phase
    phi_ = dt_ / cycle_duration_;

    // If the phase is greater than one, reset it to 0 (plus the init_phase_).
    if (phi_ - init_phase_ > 1) {
        phi_ -= 1;
        cycles_completed_ += 1;
    }
}



/* ========================================================================== */
/*                            STEP_INITIALIZATION_0                           */
/* ========================================================================== */

void StaticWalkPlanner::step_initialization_0(GeneralizedPose& gen_pose)
{
    Vector3d init_pos {0, 0, h_base_des_};

    // Base linear quantities.
    gen_pose.base_acc << 0, 0, 0;
    gen_pose.base_vel << 0, 0, 0;
    gen_pose.base_pos << 0, 0, 0.6;
    gen_pose.base_pos += phi_ / (init_phase_ / 2) * (init_pos - gen_pose.base_pos);

    // Contact feet names.
    gen_pose.contact_feet_names = all_feet_names_;

    // The swing feet quantities are empty lists since no feet are in swing phase.
    // gen_pose.feet_acc = {};
    // gen_pose.feet_vel = {};
    // gen_pose.feet_pos = {};
}



/* ========================================================================== */
/*                            STEP_INITIALIZATION_1                           */
/* ========================================================================== */

void StaticWalkPlanner::step_initialization_1(GeneralizedPose& gen_pose)
{
    Vector3d init_pos {0, 0, h_base_des_};
    Vector3d init_end_pos {base_osc_.first, - base_osc_.second, h_base_des_};
    double t_phi = (phi_ - init_phase_/2) / (init_phase_/2);

    // Base linear quantities.
    spline(init_pos, init_end_pos, t_phi,
           gen_pose.base_pos, gen_pose.base_vel, gen_pose.base_acc);

    // Contact feet names.
    gen_pose.contact_feet_names = all_feet_names_;

    // The swing feet quantities are empty lists since no feet are in swing phase.
    // gen_pose.feet_acc = {};
    // gen_pose.feet_vel = {};
    // gen_pose.feet_pos = {};
}



/* ========================================================================== */
/*                                  STEP_PLAN                                 */
/* ========================================================================== */

void StaticWalkPlanner::step_plan(GeneralizedPose& gen_pose)
{
    /* ====================== Initialization Quantities ===================== */

    // These quantities define how much the base oscillates.
    //
    //   LF      |      RF
    //           |
    //         2 | 4
    //   --------|------
    //         3 | 1
    //           |
    //   LH      |      RH

    MatrixXd r_b_osc(4,3);
    r_b_osc << - base_osc_.first, - base_osc_.second, h_base_des_,      // place the base here before lifting the left front foot
                 base_osc_.first,   base_osc_.second, h_base_des_,      // place the base here before lifting the right hind foot
               - base_osc_.first,   base_osc_.second, h_base_des_,      // place the base here before lifting the left hind foot
                 base_osc_.first, - base_osc_.second, h_base_des_;      // place the base here before lifting the right front foot

    // Position of the robot legs in the base frame
    MatrixXd abs_legs_pos(4,2);
    abs_legs_pos << + abs_leg_pos_.first, + abs_leg_pos_.second,        // left front foot
                    - abs_leg_pos_.first, - abs_leg_pos_.second,        // right hind foot
                    + abs_leg_pos_.first, - abs_leg_pos_.second,        // left hind foot
                    - abs_leg_pos_.first, + abs_leg_pos_.second;        // right front foot

    // Step vector
    Vector3d step {step_length_, 0, 0};

    // Consider a new phase independent on the initialization time
    double phi = phi_ - init_phase_;

    // Id of the foot in swing phase (id in the gait_pattern_ vector)
    int swing_foot_id = floor(4 * phi);

    // Update the position of the base and of the legs to take into account how much the robot has walked.
    r_b_osc = r_b_osc + MatrixXd::Ones(4, 1) * step * (swing_foot_id/4 + cycles_completed_);
    abs_legs_pos.row(swing_foot_id) += step.head(2) * cycles_completed_;


    /* =========================== Move COG Phase =========================== */

    // In the first part of the motion the base is moved from the old position to the new position (on the opposite side of the foot that will be raised) in order to guarantee static stability.
    if (4*phi - swing_foot_id < step_duty_factor_) {
        double phi_2 = 4*phi - swing_foot_id;

        double delta_t = step_duty_factor_;

        int previous_id;
        if (swing_foot_id - 1 < 0) {previous_id = 3;}
        else {previous_id = swing_foot_id - 1;}

        Vector3d p_i = r_b_osc.row(previous_id);
        Vector3d p_f = r_b_osc.row(swing_foot_id);

        spline(p_i, p_f, phi_2/delta_t,
               gen_pose.base_pos, gen_pose.base_vel, gen_pose.base_acc);

        std::vector<std::string> contactFeet = all_feet_names_;

        // The swing feet quantities are empty lists since no feet are in swing phase.
        // gen_pose.feet_acc = {};
        // gen_pose.feet_vel = {};
        // gen_pose.feet_pos = {};
    }

    /* ========================== Swing Foot Phase ========================== */

    // After the base has been moved to the new position that guarantees static stability, move the foot.
    else {
        /* ========================== Base Movement ========================= */

        // The desired base position, velocity, and acceleration are computed by calculating the initial and final position of the base, and by allocating the appropriate time requirements for this movement.
        // The base movement from one r_b_osc to the successive should happen in 1/4 * step_duty_factor phase.

        double phi_2 = 4*phi - swing_foot_id - step_duty_factor_;

        double delta_T = 1 - step_duty_factor_;

        // Compute the base desired quantities by using a polynomial spline. Such a spline is used in order to have null initial and final velocity and acceleration.
        Vector3d p_i = r_b_osc.row(swing_foot_id);
        Vector3d p_f = r_b_osc.row(swing_foot_id);
        p_f += step/4;

        spline(p_i, p_f, phi_2/delta_T,
               gen_pose.base_pos, gen_pose.base_vel, gen_pose.base_acc);

        // Obtain the list of feet in contact with the terrain by removing the swing_foot_id foot. This assures that the list has the correct order and is consistent with the other parts of the code.
        std::vector<std::string> contact_feet = all_feet_names_;
        contact_feet.erase(
            contact_feet.begin()
            + static_cast<int>(std::find(contact_feet.begin(), contact_feet.end(), gait_pattern_[swing_foot_id]) - contact_feet.begin())
        );

        /* ==================== Horizontal Foot Movement ==================== */

        // The desired position, velocity, and acceleration of the swing feet are obtained by considering two motions: an horizontal motion from the initial foot position to the final foot position (equal to init_foot_pos + step_vector) and a vertical motion that first raises and than lowers the foot.

        // Leg initial and final horizontal position
        Vector3d leg_init_pos {abs_legs_pos(swing_foot_id, 0),
                               abs_legs_pos(swing_foot_id, 1),
                               0};
        Vector3d leg_end_pos {abs_legs_pos(swing_foot_id, 0) + step_length_,
                              abs_legs_pos(swing_foot_id, 1),
                              0};

        // Horizontal part of the desired position, velocity, and acceleration of the swing feet
        spline(leg_init_pos, leg_end_pos, phi_2/delta_T,
               gen_pose.feet_pos, gen_pose.feet_vel, gen_pose.feet_acc);

        /* ===================== Vertical Foot Movement ===================== */

        VectorXd r_s_ddot_des_2;
        VectorXd r_s_dot_des_2;
        VectorXd r_s_des_2;

        // Raise the foot during the first half
        if (4*phi - swing_foot_id < (1 - step_duty_factor_)/2 + step_duty_factor_) {
            phi_2 = 4*phi - swing_foot_id - step_duty_factor_;

            delta_T = (1 - step_duty_factor_) / 2;

            leg_init_pos << abs_legs_pos(swing_foot_id, 0),
                            abs_legs_pos(swing_foot_id, 1),
                            0;
            leg_end_pos <<  abs_legs_pos(swing_foot_id, 0),
                            abs_legs_pos(swing_foot_id, 1),
                            step_height_;

            spline(leg_init_pos, leg_end_pos, phi_2/delta_T,
                    r_s_des_2, r_s_dot_des_2, r_s_ddot_des_2);
        }
        // Lower the foot during the second half
        else {
            phi_2 = 4*phi - swing_foot_id - step_duty_factor_ - (1 - step_duty_factor_)/2;

            delta_T = (1 - step_duty_factor_) / 2;

            leg_init_pos << abs_legs_pos(swing_foot_id, 0),
                            abs_legs_pos(swing_foot_id, 1),
                            step_height_;
            leg_end_pos <<  abs_legs_pos(swing_foot_id, 0) + step_length_,
                            abs_legs_pos(swing_foot_id, 1),
                            - desired_foot_penetration_;

            spline(leg_init_pos, leg_end_pos, phi_2/delta_T,
                    r_s_des_2, r_s_dot_des_2, r_s_ddot_des_2);
        }

        // Superimpose the vertical movement of the foot to the horizontal movement
        gen_pose.feet_pos(2) += r_s_des_2(2);
        gen_pose.feet_vel(2) += r_s_dot_des_2(2);
        gen_pose.feet_acc(2) += r_s_ddot_des_2(2);
    }
}



/* ========================================================================== */
/*                                   SPLINE                                   */
/* ========================================================================== */

void StaticWalkPlanner::spline(
    Eigen::Ref<Vector3d> p_i, Eigen::Ref<Vector3d> p_f, double t,
    Eigen::Ref<Vector3d> p_t, Eigen::Ref<Vector3d> v_t, Eigen::Ref<Vector3d> a_t
) {
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