/* ========================================================================== */
/*                                 DESCRIPTION                                */
/* ========================================================================== */

/*

*/



/* ========================================================================== */
/*                              IMPORT LIBRARIES                              */
/* ========================================================================== */

#include "pinocchio/algorithm/kinematics.hpp"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>



/* ========================================================================== */
/*                                    CODE                                    */
/* ========================================================================== */

namespace robot_wrapper {

class RobotModel {
    public:
        RobotModel(std::string robot_name);

        void compute_EOM(Eigen::VectorXd& q, Eigen::VectorXd& v);

        void compute_second_order_FK(Eigen::VectorXd& q, Eigen::VectorXd& v);

        void get_Jc(Eigen::MatrixXd& Jc);

        void get_Jb(Eigen::MatrixXd& Jb);

        void get_Js(Eigen::MatrixXd& Js);

        void get_Jc_dot_times_v(Eigen::VectorXd& Jc_dot_times_v);

        void get_Jb_dot_times_v(Eigen::VectorXd& Jb_dot_times_v);

        void get_Js_dot_times_v(Eigen::VectorXd& Js_dot_times_v);

        void get_oRb(Eigen::MatrixXd& oRb);

        void get_r_s(Eigen::VectorXd& r_s);

        const pinocchio::Model& get_model() const { return model; }
        
        const pinocchio::Data& get_data() const { return data; }

        void set_contact_feet_names(std::vector<std::string> contact_feet_names) {
            this->contact_feet_names = contact_feet_names;
        }

        void compute_swing_feet_names();

    private:
        pinocchio::Model model;

        pinocchio::Data data;

        std::string urdf_path;

        std::vector<std::string> feet_names;

        std::vector<std::string> contact_feet_names;
        
        std::vector<std::string> swing_feet_names;

        Eigen::VectorXd feet_displacement;
};

} // robot_wrapper