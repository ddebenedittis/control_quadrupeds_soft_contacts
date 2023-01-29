#include "robot_model/robot_model.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include <ryml_std.hpp> // optional header. BUT when used, needs to be included BEFORE ryml.hpp
#include <ryml.hpp>
#include <c4/format.hpp>



namespace robot_wrapper {

/* ========================================================================== */
/*                           RYML-RELATED FUNCTIONS                           */
/* ========================================================================== */

// Helper functions for sample_parse_file()
template<class CharContainer> CharContainer file_get_contents(const char *filename);
template<class CharContainer> size_t        file_get_contents(const char *filename, CharContainer *v);

// Helper functions for sample_parse_file()

C4_SUPPRESS_WARNING_MSVC_WITH_PUSH(4996) // fopen: this function or variable may be unsafe
/// Load a file from disk and return a newly created CharContainer */
template<class CharContainer>
size_t file_get_contents(const char *filename, CharContainer *v)
{
    ::FILE *fp = ::fopen(filename, "rb");
    C4_CHECK_MSG(fp != nullptr, "could not open file");
    ::fseek(fp, 0, SEEK_END);
    long sz = ::ftell(fp);
    v->resize(static_cast<typename CharContainer::size_type>(sz));
    if (sz) {
        ::rewind(fp);
        size_t ret = ::fread(&(*v)[0], 1, v->size(), fp);
        C4_CHECK(ret == (size_t)sz);
    }
    ::fclose(fp);
    return v->size();
}

/// Load a file from disk into an existing CharContainer */
template<class CharContainer>
CharContainer file_get_contents(const char *filename)
{
    CharContainer cc;
    file_get_contents(filename, &cc);
    return cc;
}



/* ========================================================================== */
/*                              ROBOTMODEL CLASS                              */
/* ========================================================================== */

/* ========================= RobotModel Constructor ========================= */

RobotModel::RobotModel(const std::string& robot_name)
: feet_names(4),
  feet_displacement(3)
{
    // Location of the file containing some info on the robots
    std::string file_path = ament_index_cpp::get_package_share_directory("robot_model") + std::string{"/robots/all_robots.yaml"};

    // Parse the yaml file
    std::string contents = file_get_contents<std::string>(file_path.c_str());
    ryml::Tree tree = ryml::parse_in_arena(ryml::to_csubstr(contents)); // immutable (csubstr) overload

    ryml::NodeRef root = tree.rootref();
    ryml::NodeRef root_robot;

    // Find where the robot name is in the file.
    for(ryml::NodeRef n : root.children()) {
        if (ryml::to_csubstr(robot_name) == n.key()) {
            root_robot = n;
            break;
        }
    }

    std::string pkg_name;
    ryml::from_chars(root_robot["pkg_name"].val(), &pkg_name);
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(pkg_name);

    // Populate the urdf_path attribute
    ryml::from_chars(root_robot["urdf_path"].val(), &this->urdf_path);
    this->urdf_path.insert(0, package_share_directory);

    // Populate the feet_names attribute
    for (int i=0; i<4; i++) {
        ryml::from_chars(root_robot["feet_names"][i].val(), &this->feet_names[i]);
    }

    // Populate the feet_displacement attribute. Local scope for the temp_string variable.
    {
        std::string temp_string;

        for(int i=0; i<3; i++) {
            ryml::from_chars(root_robot["ankle_feet_displacement"][i].val(), &temp_string);
            this->feet_displacement(i) = std::stof(temp_string);
        }
    }

    // Load the urdf model
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(urdf_path, root_joint, model);

    // Create the data required by the algorithms
    pinocchio::Data data(model);
    this->data = data;
}


/* =============================== compute_EOM ============================== */

void RobotModel::compute_EOM(const Eigen::VectorXd& q, const Eigen::VectorXd& v)
{
    // Update the joint placements
    pinocchio::forwardKinematics(model, data, q);

    // Computes the full model Jacobian
    pinocchio::computeJointJacobians(model, data, q);

    // Update the frame placements
    pinocchio::updateFramePlacements(model, data);

    // Compute the upper part of the joint space inertia matrix
    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // Compute the nonlinear effects vector (Coriolis, centrifugal and gravitational effects)
    pinocchio::nonLinearEffects(model, data, q, v);
}


/* ========================= compute_second_order_FK ======================== */

void RobotModel::compute_second_order_FK(const Eigen::VectorXd& q, const Eigen::VectorXd& v)
{
    // Update the joint accelerations
    pinocchio::forwardKinematics(model, data, q, v, Eigen::VectorXd::Zero(model.nv));

    // Computes the full model Jacobian variations with respect to time
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);
}


/* =============================== compute_Jc =============================== */

void RobotModel::get_Jc(Eigen::MatrixXd& Jc)
{
    // Initialize a temp Jacobian that must be used to store the contact jacobian of a contact foot.
    // Jc is the stack of J_temp of all the contact feet.
    Eigen::MatrixXd J_temp(6, model.nv);

    // Compute the stack of the contact Jacobians
    for (size_t i = 0; i < contact_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = model.getFrameId(contact_feet_names[i]);

        J_temp.setZero();
        
        pinocchio::getFrameJacobian(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

        Jc.block(3*i, 0, 3, model.nv) = J_temp.topRows(3);
    }
}


/* =============================== compute_Jb =============================== */

void RobotModel::get_Jb(Eigen::MatrixXd& Jb)
{
    Jb.setZero();

    pinocchio::FrameIndex base_id = 1;

    pinocchio::getFrameJacobian(model, data, base_id, pinocchio::LOCAL_WORLD_ALIGNED, Jb);
}


/* =============================== compute_Js =============================== */

void RobotModel::get_Js(Eigen::MatrixXd& Js)
{
    // Initialize a temp Jacobian that must be used to store the swing feet jacobian.
    // Js is the stack of J_temp of all the swing feet.
    Eigen::MatrixXd J_temp(6, model.nv);

    // Compute the stack of the swing jacobians.
    for (size_t i = 0; i < swing_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = model.getFrameId(swing_feet_names[i]);

        J_temp.setZero();
        
        pinocchio::getFrameJacobian(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

        Js.block(3*i, 0, 3, model.nv) = J_temp.topRows(3);
    }
}


/* ========================= compute_Jc_dot_times_v ========================= */

void RobotModel::get_Jc_dot_times_v(Eigen::VectorXd& Jc_dot_times_v)
{
    for (size_t i = 0; i < contact_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = model.getFrameId(contact_feet_names[i]);

        Jc_dot_times_v.segment(0+3*i, 3) = pinocchio::getFrameClassicalAcceleration(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();
    }
}


/* ========================= compute_Jc_dot_times_v ========================= */

void RobotModel::get_Jb_dot_times_v(Eigen::VectorXd& Jb_dot_times_v)
{
    pinocchio::FrameIndex base_id = 1;

    Jb_dot_times_v = pinocchio::getClassicalAcceleration(model, data, base_id, pinocchio::LOCAL_WORLD_ALIGNED).toVector();
}


/* ========================= compute_Js_dot_times_v ========================= */

void RobotModel::get_Js_dot_times_v(Eigen::VectorXd& Js_dot_times_v)
{
    for (size_t i = 0; i < swing_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = model.getFrameId(swing_feet_names[i]);

        Js_dot_times_v.segment(0+3*i, 3) = pinocchio::getFrameClassicalAcceleration(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();
    }
}


/* =============================== Compute_oRb ============================== */

void RobotModel::get_oRb(Eigen::Matrix3d& oRb)
{
    pinocchio::FrameIndex base_id = 1;

    oRb = data.oMi[base_id].rotation();
}


/* ================================= get_r_s ================================ */

void RobotModel::get_r_s(Eigen::VectorXd& r_s)
{
    for (size_t i = 0; i < swing_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = model.getFrameId(swing_feet_names[i]);

        r_s.segment(3*i, 3) = data.oMf[frame_id].translation() + feet_displacement;
    }
}


/* ============================ get_feet_positions =========================== */

Eigen::VectorXd RobotModel::get_feet_positions()
{
    Eigen::VectorXd feet_position(12);

    for (size_t i = 0; i < feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = model.getFrameId(feet_names[i]);

        feet_position.segment(3*i, 3) = data.oMf[frame_id].translation() + feet_displacement;
    }

    return feet_position;
}


/* =========================== get_feet_velocities ========================== */

Eigen::VectorXd RobotModel::get_feet_velocities(const Eigen::VectorXd& v)
{
    Eigen::VectorXd feet_velocities(12);
    Eigen::MatrixXd J_temp(6, model.nv);

    for (int i=0; i<4; i++) {
        pinocchio::FrameIndex frame_id = model.getFrameId(this->feet_names[i]);

        J_temp.setZero();

        pinocchio::getFrameJacobian(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

        feet_velocities.segment(0+3*i, 3) = J_temp.topRows(3) * v;
    }

    return feet_velocities;
}


/* ======================== compute_swing_feet_names ======================== */

void RobotModel::set_swing_feet_names()
{
    swing_feet_names = {};

    for (size_t i = 0; i < 4; i++) {
        if ( std::find(contact_feet_names.begin(), contact_feet_names.end(), feet_names[i]) == contact_feet_names.end() ) {
            // The foot name is NOT a member of the contact_feet_names vector, hence it is a swing foot.
            swing_feet_names.push_back(feet_names[i]);
        }
    }
}

} // robot_wrapper