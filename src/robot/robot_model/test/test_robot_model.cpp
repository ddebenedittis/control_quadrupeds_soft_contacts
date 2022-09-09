#include <robot_model/robot_model.hpp>

#include "pinocchio/algorithm/joint-configuration.hpp"

#include <iostream>



int main()
{
    using namespace pinocchio;
    using namespace std;

    std::string robot_name = "anymal_c";

    robot_wrapper::RobotModel rob(robot_name);

    Model model = rob.get_model();
    Data data = rob.get_data();
  
    // Sample a random configuration
    // Eigen::VectorXd q = randomConfiguration(model);
    Eigen::VectorXd q = neutral(model);
    std::cout << "q: " << q.transpose() << std::endl;
 
    // Perform the forward kinematics over the kinematic tree
    forwardKinematics(model,data,q);
 
    // Print out the placement of each joint of the kinematic tree
    for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
        std::cout << std::setw(24) << std::left
                  << model.names[joint_id] << ": "
                  << std::fixed << std::setprecision(2)
                  << data.oMi[joint_id].translation().transpose()
                  << std::endl;

    std::vector<std::string> contact_feet_names{"LF_FOOT", "RF_FOOT", "RH_FOOT"};
    int nc = contact_feet_names.size();
    int nv = model.nv;
    
    rob.set_feet_names(contact_feet_names);

    Eigen::VectorXd v;
    v = Eigen::VectorXd::Zero(q.size() - 1);

    rob.compute_EOM(q, v);
    cout << "compute_EOM successfull\n";

    Eigen::MatrixXd Jc(3*nc, nv);
    rob.get_Jc(Jc);
    cout << "compute_Jc successfull\n";

    Eigen::MatrixXd Jb(6, nv);
    rob.get_Jb(Jb);
    cout << "compute_Jb successfull\n";

    Eigen::MatrixXd Js(12-3*nc, nv);
    rob.get_Js(Js);
    cout << "compute_Js successfull\n";

    Eigen::VectorXd get_Jc_dot_times_v(3*nc);
    rob.get_Jc_dot_times_v(get_Jc_dot_times_v);
    cout << "get_Jc_dot_times_v successfull\n";

    Eigen::VectorXd get_Jb_dot_times_v(6);
    rob.get_Jb_dot_times_v(get_Jb_dot_times_v);
    cout << "get_Jb_dot_times_v successfull\n";

    Eigen::VectorXd get_Js_dot_times_v(12-3*nc);
    rob.get_Js_dot_times_v(get_Js_dot_times_v);
    cout << "get_Js_dot_times_v successfull\n";

    Eigen::Matrix3d oRb;
    rob.get_oRb(oRb);
    cout << "get_oRb successfull\n";

    Eigen::VectorXd r_s(12-3*nc);
    rob.get_r_s(r_s);
    cout << "get_r_s successfull\n";
    cout << r_s << std::endl;
}