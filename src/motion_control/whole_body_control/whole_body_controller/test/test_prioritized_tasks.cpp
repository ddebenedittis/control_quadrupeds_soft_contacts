#include <whole_body_controller/prioritized_tasks.hpp>

#include <iostream>



int main()
{
    using namespace std;
    using namespace Eigen;

    string robot_name = "anymal_c";

    float dt = 1./400.;

    wbc::PrioritizedTasks prio_tasks(robot_name, dt);

    cout << "PrioritizedTasks constructed successfully\n";

    MatrixXd A;
    VectorXd b;
    MatrixXd C;
    VectorXd d;

    VectorXd q = VectorXd::Zero(19);
    q(6) = 1;
    VectorXd v = VectorXd::Zero(18);

    wbc::GeneralizedPose gen_pose;
    // gen_pose.feet_acc = VectorXd::Zero(6);
    // gen_pose.feet_vel = VectorXd::Zero(6);
    // gen_pose.feet_pos = VectorXd::Zero(6);
    // gen_pose.contact_feet_names = {"LF_FOOT", "LH_FOOT"};
    gen_pose.feet_acc = VectorXd::Zero(0);
    gen_pose.feet_vel = VectorXd::Zero(0);
    gen_pose.feet_pos = VectorXd::Zero(0);
    gen_pose.contact_feet_names = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

    // VectorXd d_k1 = VectorXd::Ones(6);
    // VectorXd d_k2 = VectorXd::Ones(6);
    VectorXd d_k1 = VectorXd::Ones(12);
    VectorXd d_k2 = VectorXd::Ones(12);

    prio_tasks.reset(q, v, gen_pose.contact_feet_names);

    cout << "reset successfull\n";

    prio_tasks.compute_task_p(0, A, b, C, d, gen_pose, d_k1, d_k2);

    cout << "Priority 0 successfull\n";

    prio_tasks.compute_task_p(1, A, b, C, d, gen_pose, d_k1, d_k2);

    cout << "Priority 1 successfull\n";

    prio_tasks.compute_task_p(2, A, b, C, d, gen_pose, d_k1, d_k2);

    cout << "Priority 2 successfull\n";

    prio_tasks.compute_task_p(3, A, b, C, d, gen_pose, d_k1, d_k2);

    cout << "Priority 3 successfull\n";

    prio_tasks.compute_task_p(4, A, b, C, d, gen_pose, d_k1, d_k2);

    cout << "Priority 4 successfull\n";

    prio_tasks.get_all_feet_names();

    cout << "get_all_feet_names successfull\n";

    prio_tasks.get_max_priority();

    cout << "get_max_priority successfull\n";

    return 0;
}