#include "whole_body_controller/control_tasks.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include <iostream>

int main()
{
    std::string robot_name = "anymal_c";

    float dt = 1./400.;

    wbc::ControlTasks control_tasks(robot_name, dt);

    Eigen::VectorXd q = pinocchio::randomConfiguration(control_tasks.get_model());
    Eigen::VectorXd v = Eigen::VectorXd::Zero(q.size() - 1);

    // std::vector<std::string> contact_feet_names{"LF_FOOT", "RF_FOOT", "RH_FOOT"};
    // std::vector<std::string> contact_feet_names{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    std::vector<std::string> contact_feet_names{};

    std::cout << "Construction successful\n";

    control_tasks.reset(q, v, contact_feet_names);

    std::cout << "Reset successful\n";

    int nv = control_tasks.get_nv();
    int nc = control_tasks.get_nc();
    int nF = control_tasks.get_nF();
    int nd = control_tasks.get_nd();

    int nx = nv + nF + nd;

    Eigen::MatrixXd A(100,100);
    Eigen::VectorXd b(100);
    Eigen::MatrixXd C(100,100);
    Eigen::VectorXd d(100);

    Eigen::Vector3d base_pos;
    base_pos << 0,0,0;

    Eigen::Vector4d base_orient;
    base_orient << 0,0,1,0;

    Eigen::VectorXd swing_feet_xxx;
    // swing_feet_xxx << 0,0,0;
    // swing_feet_xxx = Eigen::VectorXd::Zero(0);
    swing_feet_xxx = Eigen::VectorXd::Zero(12);

    // Eigen::VectorXd d_k = Eigen::VectorXd::Zero(9);
    // Eigen::VectorXd d_k = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd d_k = Eigen::VectorXd::Zero(0);

    control_tasks.task_floating_base_eom(A.topLeftCorner(6, nx), b.head(6));
    std::cout << "Floating base EOM successful" << std::endl;

    control_tasks.task_torque_limits(C.topLeftCorner(2*(nv-6), nx), d.head(2*(nv-6)));
    std::cout << "Torque limits successful" << std::endl;

    control_tasks.task_friction_Fc_modulation(C.topLeftCorner(6*(nv-6), nx), d.head(6*(nv-6)));
    std::cout << "Friction limits and Fc modulation successful" << std::endl;

    control_tasks.task_linear_motion_tracking(
        A.topLeftCorner(3, nx), b.head(3),
        base_pos,base_pos,base_pos
    );
    std::cout << "Linear motion tracking successful" << std::endl;

    control_tasks.task_angular_motion_tracking(
        A.topLeftCorner(3, nx), b.head(3),
        base_pos,base_orient
    );
    std::cout << "Angular motion tracking successful" << std::endl;

    control_tasks.task_swing_feet_tracking(
        A.topLeftCorner(12 - 3*nc, nx), b.head(12 - 3*nc),
        swing_feet_xxx,swing_feet_xxx,swing_feet_xxx
    );
    std::cout << "Swing feet motion tracking successful" << std::endl;

    control_tasks.task_contact_constraints_soft_kv(
        A, b,
        C, d,
        d_k, d_k
    );
    std::cout << "Contact constraints task successfull" << std::endl;

    control_tasks.task_energy_forces_minimization(
        A, b
    );
    std::cout << "Energy and forces minimization successfull" << std::endl;
}