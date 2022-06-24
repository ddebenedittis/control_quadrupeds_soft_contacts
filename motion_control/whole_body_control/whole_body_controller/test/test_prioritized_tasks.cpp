#include <whole_body_controller/prioritized_tasks.hpp>

#include <iostream>

int main()
{
    using namespace std;
    using namespace Eigen;

    string robot_name = "anymal_c";

    wbc::PrioritizedTasks prio_tasks(robot_name);

    cout << "PrioritizedTasks constructed successfully\n";

    MatrixXd A;
    VectorXd b;
    MatrixXd C;
    VectorXd d;

    wbc::GeneralizedPose gen_pose;
    gen_pose.feet_acc = VectorXd::Zero(6);
    gen_pose.feet_vel = VectorXd::Zero(6);
    gen_pose.feet_pos = VectorXd::Zero(6);
    gen_pose.contact_feet_names = {"LF_FOOT", "LH_FOOT"};

    VectorXd d_k1 = VectorXd::Ones(6);
    VectorXd d_k2 = VectorXd::Ones(6);

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

    return 0;
}