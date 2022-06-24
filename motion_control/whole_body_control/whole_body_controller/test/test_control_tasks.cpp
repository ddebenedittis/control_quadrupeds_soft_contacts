#include "whole_body_controller/control_tasks.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include <iostream>

int main()
{
    std::string robot_name = "anymal_c";

    wbc::ControlTasks control_tasks(robot_name);

    Eigen::VectorXd q = pinocchio::randomConfiguration(control_tasks.robot_model.get_model());
    Eigen::VectorXd v = Eigen::VectorXd::Zero(q.size() - 1);

    std::vector<std::string> contact_feet_names{"LF_FOOT", "RF_FOOT", "RH_FOOT"};

    std::cout << "Construction successful\n";

    control_tasks.reset(q, v, contact_feet_names);

    std::cout << "Reset successful\n";

    int nv = control_tasks.nv;
    int nc = control_tasks.nc;
    int nF = control_tasks.nF;
    int nd = control_tasks.nd;

    int nx = nv + nF + nd;

    Eigen::MatrixXd A(100,100);
    Eigen::VectorXd b(100);
    Eigen::MatrixXd C(100,100);
    Eigen::VectorXd d(100);

    Eigen::Vector3d zzz;
    zzz << 0,0,0;

    Eigen::Vector4d qqqq;
    qqqq << 0,0,1,0;

    control_tasks.task_floating_base_eom(A.topLeftCorner(6, nx), b.head(6));
    std::cout << "Floating base EOM successful\n";

    control_tasks.task_torque_limits(C.topLeftCorner(2*(nv-6), nx), d.head(2*(nv-6)));
    std::cout << "Torque limits successful\n";

    control_tasks.task_friction_Fc_modulation(C.topLeftCorner(6*(nv-6), nx), d.head(6*(nv-6)));
    std::cout << "Friction limits and Fc modulation successful\n";

    control_tasks.task_linear_motion_tracking(
        A.topLeftCorner(3, nx), b.head(3),
        zzz,zzz,zzz
    );
    std::cout << "Linear motion tracking successful\n";

    control_tasks.task_angular_motion_tracking(
        A.topLeftCorner(3, nx), b.head(3),
        zzz,qqqq
    );
    std::cout << "Angular motion tracking successful\n";

    control_tasks.task_linear_motion_tracking(
        A.topLeftCorner(12 - 3*nc, nx), b.head(12 - 3*nc),
        zzz,zzz,zzz
    );
    std::cout << "Swing feet motion tracking successful\n";
}