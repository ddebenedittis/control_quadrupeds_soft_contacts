#include "whole_body_controller/whole_body_controller.hpp"

#include <iostream>



int main()
{
    using namespace wbc;
    using namespace Eigen;
    using namespace std;

    std::string robot_name = "anymal_c";
    float dt = 1./400.;

    WholeBodyController wbc(robot_name, dt);

    cout << "wbc constructed successfully" << endl;

    VectorXd q = VectorXd::Zero(19);
    q(6) = 1;
    q(12) = 0.2; q(15) = -0.33;
    VectorXd v = VectorXd::Zero(18);
    v(9) = - 2; v(13) = - 1;

    GeneralizedPose gen_pose;
    // gen_pose.feet_acc = VectorXd::Zero(6);
    // gen_pose.feet_vel = VectorXd::Zero(6);
    // gen_pose.feet_pos = VectorXd::Zero(6);
    // gen_pose.contact_feet_names = {"LF_FOOT", "LH_FOOT"};

    gen_pose.feet_acc = VectorXd::Zero(0);
    gen_pose.feet_vel = VectorXd::Zero(0);
    gen_pose.feet_pos = VectorXd::Zero(0);
    gen_pose.contact_feet_names = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

    // wbc.step(q, v, gen_pose);

    // cout << "step 1 completed successfully\n";

    // Eigen::VectorXd tau = wbc.get_tau_opt();

    // cout << tau << "\n" << endl;

    // cout << "get_torques completed successfully" << std::endl;

    gen_pose.base_pos = {0, 0, 0.55};
    gen_pose.contact_feet_names = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

    wbc.set_kp_b_pos(100 * Eigen::Vector3d(1,1,1));
    wbc.set_kd_b_pos( 10 * Eigen::Vector3d(1,1,1));

    wbc.set_kp_b_ang(150 * Eigen::Vector3d(1,1,1));
    wbc.set_kd_b_ang( 35 * Eigen::Vector3d(1,1,1));

    wbc.set_kp_s_pos(150 * Eigen::Vector3d(1,1,1));
    wbc.set_kd_s_pos( 30 * Eigen::Vector3d(1,1,1));

    wbc.set_Kp_terr(1000 * Eigen::Vector3d(1,1,1));
    wbc.set_Kd_terr(1000 * Eigen::Vector3d(1,1,1));

    q << -4.00332046e-06,  6.52628557e-06,  6.31907932e-01, -3.91157384e-05,
          1.05449352e-04, -4.40777218e-07,  9.99999994e-01, -1.32374422e-02,
          3.69140336e-02, -6.10268612e-02, -1.37317625e-02, -3.94673767e-02,
          6.48098113e-02,  1.32987136e-02,  3.70337976e-02, -6.11880500e-02,
          1.38186318e-02, -3.96016448e-02,  6.50032111e-02;
    v << -2.02562655e-04,  2.61344500e-04,  1.00147370e-01, -5.18886597e-03,
          1.85864755e-02,  4.94709400e-04, -1.03050612e+00,  1.14787118e+00,
         -1.60779492e+00, -1.07193201e+00, -1.29660846e+00,  1.77220742e+00,
          1.03693607e+00,  1.15892654e+00, -1.62150350e+00,  1.08674935e+00,
         -1.30117959e+00,  1.77660008e+00;
    wbc.step(q, v, gen_pose);

    Eigen::VectorXd tau = wbc.get_tau_opt();

    cout << tau << "\n" << endl;

    q << -4.00332046e-06,  6.52628557e-06,  6.31907932e-01, -3.91157384e-05,
          1.05449352e-04, -4.40777218e-07,  9.99999994e-01, -2.35676782e-02,
          4.83720489e-02, -7.69445160e-02, -2.44840524e-02, -5.24114312e-02,
          8.23529430e-02,  2.37027173e-02,  4.86005011e-02, -7.72404283e-02,
          2.46973234e-02, -5.25854122e-02,  8.25873295e-02;
    v << -2.02562655e-04,  2.61344500e-04,  1.00147370e-01, -5.18886597e-03,
          1.85864755e-02,  4.94709400e-04, -1.03479344e+00,  1.14478042e+00,
         -1.58170516e+00, -1.07751682e+00, -1.29324105e+00,  1.74302129e+00,
          1.04254360e+00,  1.15557588e+00, -1.59507147e+00,  1.08930535e+00,
         -1.29698431e+00,  1.74699197e+00;
    wbc.step(q, v, gen_pose);

    tau = wbc.get_tau_opt();

    cout << tau << "\n" << endl;

    q << -4.00332046e-06,  6.52628557e-06,  6.31907932e-01, -3.91157384e-05,
          1.05449352e-04, -4.40777218e-07,  9.99999994e-01, -2.35676782e-02,
          4.83720489e-02, -7.69445160e-02, -2.44840524e-02, -5.24114312e-02,
          8.23529430e-02,  2.37027173e-02,  4.86005011e-02, -7.72404283e-02,
          2.46973234e-02, -5.25854122e-02,  8.25873295e-02;
    v << -2.02562655e-04,  2.61344500e-04,  1.00147370e-01, -5.18886597e-03,
          1.85864755e-02,  4.94709400e-04, -1.03479344e+00,  1.14478042e+00,
         -1.58170516e+00, -1.07751682e+00, -1.29324105e+00,  1.74302129e+00,
          1.04254360e+00,  1.15557588e+00, -1.59507147e+00,  1.08930535e+00,
         -1.29698431e+00,  1.74699197e+00;
    wbc.step(q, v, gen_pose);

    tau = wbc.get_tau_opt();

    cout << tau << "\n" << endl;

    q << -7.30421128e-06,  4.25434169e-06,  6.32317004e-01, -4.29902655e-05,
          2.48132853e-04, -1.43107451e-06,  9.99999968e-01, -2.35676782e-02,
          4.83720489e-02, -7.69445160e-02, -2.44840524e-02, -5.24114312e-02,
          8.23529430e-02,  2.37027173e-02,  4.86005011e-02, -7.72404283e-02,
          2.46973234e-02, -5.25854122e-02,  8.25873295e-02;
    v << -6.32633000e-05, -1.26162398e-04, -3.15400257e-02, -5.78185701e-03,
          1.69523028e-02,  9.11627853e-04, -1.03479344e+00,  1.14478042e+00,
         -1.58170516e+00, -1.07751682e+00, -1.29324105e+00,  1.74302129e+00,
          1.04254360e+00,  1.15557588e+00, -1.59507147e+00,  1.08930535e+00,
         -1.29698431e+00,  1.74699197e+00;

    gen_pose.feet_acc = VectorXd::Zero(6);
    gen_pose.feet_vel = VectorXd::Zero(6);
    gen_pose.feet_pos = VectorXd::Zero(6);
    gen_pose.contact_feet_names = {"LF_FOOT", "LH_FOOT"};
    
    wbc.step(q, v, gen_pose);

    tau = wbc.get_tau_opt();

    cout << tau << "\n" << endl;

    return 0;
}