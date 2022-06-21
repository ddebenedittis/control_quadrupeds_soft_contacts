#include <robot_model/robot_model.hpp>

#include "pinocchio/algorithm/joint-configuration.hpp"

int main() {
    using namespace pinocchio;

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
}