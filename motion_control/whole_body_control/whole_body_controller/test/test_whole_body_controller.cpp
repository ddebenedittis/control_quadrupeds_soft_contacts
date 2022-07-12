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
    gen_pose.feet_acc = VectorXd::Zero(6);
    gen_pose.feet_vel = VectorXd::Zero(6);
    gen_pose.feet_pos = VectorXd::Zero(6);
    gen_pose.contact_feet_names = {"LF_FOOT", "LH_FOOT"};

    wbc.step(q, v, gen_pose);

    cout << "step completed successfully\n";

    Eigen::VectorXd tau = wbc.get_tau_opt();

    cout << tau << endl;

    cout << "get_torques completed successfully" << std::endl;

    return 0;
}