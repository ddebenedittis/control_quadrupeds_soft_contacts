#include "hqp_controller/hqp_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"



namespace hqp_controller {

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;



/* ========================================================================== */
/*                                HQPCONTROLLER                               */
/* ========================================================================== */

HQPController::HQPController()
: controller_interface::ControllerInterface(),
  wbc("anymal_c", 1/100) {}


/* ================================= On_init ================================ */

CallbackReturn HQPController::on_init()
{
    try {
        auto_declare<std::string>("robot_name", std::string());
        auto_declare<double>("sample_time", double());
    }
    catch(const std::exception& e) {
        fprintf(stderr,"Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}


/* ===================== Command_interface_configuration ==================== */

InterfaceConfiguration HQPController::command_interface_configuration() const
{
    InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
    for (const auto& joint : joint_names_) {
        command_interfaces_config.names.push_back(joint + "/" + HW_IF_EFFORT);
    }

    return command_interfaces_config;
}


/* ====================== State_interface_configuration ===================== */

InterfaceConfiguration HQPController::state_interface_configuration() const
{
    InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = interface_configuration_type::INDIVIDUAL;
    for (const auto& joint : joint_names_) {
        state_interfaces_config.names.push_back(joint + "/" + HW_IF_POSITION);
        state_interfaces_config.names.push_back(joint + "/" + HW_IF_VELOCITY);
    }
    
    return state_interfaces_config;
}


/* ============================== On_configure ============================== */

CallbackReturn HQPController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
    std::string robot_name = get_node()->get_parameter("robot_name").as_string();
    if (robot_name.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(),"'robot_name' parameter is empty");
        return CallbackReturn::ERROR;
    }

    double dt = get_node()->get_parameter("sample_time").as_double();
    if (dt <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'sample_time' parameter is <= 0");
        return CallbackReturn::ERROR;
    }

    wbc = wbc::WholeBodyController(robot_name, dt);

    q_.resize(wbc.get_nv() + 1);
    q_(6) = 1;
    v_.resize(wbc.get_nv());
    des_gen_pose_.feet_pos.resize(12);
    des_gen_pose_.feet_vel.resize(12);
    des_gen_pose_.feet_acc.resize(12);

    joint_state_subscription_ = get_node()->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/gazebo/link_states", rclcpp::SystemDefaultsQoS(),
        [this](const gazebo_msgs::msg::LinkStates::SharedPtr msg) -> void
        {
            int base_id = 1;

            geometry_msgs::msg::Point pos = msg->pose[base_id].position;
            geometry_msgs::msg::Quaternion orient = msg->pose[base_id].orientation;

            geometry_msgs::msg::Vector3 lin = msg->twist[base_id].linear;
            geometry_msgs::msg::Vector3 ang = msg->twist[base_id].angular;

            q_.head(7) << pos.x, pos.y, pos.z,
                          orient.x, orient.y, orient.z, orient.w;

            v_.head(6) << lin.x, lin.y, lin.z,
                          ang.x, ang.y, ang.z;
        }
    );

    desired_generalized_pose_subscription_ = get_node()->create_subscription<generalized_pose_msgs::msg::DesiredGeneralizedPose>(
        "/robot/desired_generalized_pose", rclcpp::SystemDefaultsQoS(),
        [this](const generalized_pose_msgs::msg::DesiredGeneralizedPose::SharedPtr msg) -> void
        {
            des_gen_pose_.base_acc << msg->base_acc.x, msg->base_acc.y, msg->base_acc.z;
            des_gen_pose_.base_vel << msg->base_vel.x, msg->base_vel.y, msg->base_vel.z;
            des_gen_pose_.base_pos << msg->base_pos.x, msg->base_pos.y, msg->base_pos.z;

            des_gen_pose_.base_angvel << msg->base_angvel.x, msg->base_angvel.y, msg->base_angvel.z;
            des_gen_pose_.base_quat << msg->base_quat.x, msg->base_quat.y, msg->base_quat.z, msg->base_quat.w;

            des_gen_pose_.feet_acc = Eigen::VectorXd::Map(&msg->feet_acc[0], msg->feet_acc.size());
            des_gen_pose_.feet_vel = Eigen::VectorXd::Map(&msg->feet_vel[0], msg->feet_vel.size());
            des_gen_pose_.feet_pos = Eigen::VectorXd::Map(&msg->feet_pos[0], msg->feet_pos.size());

            des_gen_pose_.contact_feet_names.assign(&msg->contact_feet[0], &msg->contact_feet[msg->contact_feet.size()]);
        }
    );

    return CallbackReturn::SUCCESS;
}


/* =============================== On_activate ============================== */

CallbackReturn HQPController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    return CallbackReturn::SUCCESS;
}


/* ============================== On_deactivate ============================= */

CallbackReturn HQPController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    return CallbackReturn::SUCCESS;
}


/* ================================= Update ================================= */

controller_interface::return_type HQPController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period
) {
    for (uint i=0; i<joint_names_.size(); i++) {
        q_(i+7) = state_interfaces_[2*i].get_value();
        v_(i+6) = state_interfaces_[2*i+1].get_value();
    }
    
    wbc.step(q_, v_, des_gen_pose_);

    tau_ = wbc.get_tau_opt();

    // Send effort command
    for (uint i=0; i<joint_names_.size(); i++) {
        command_interfaces_[i].set_value(tau_(i));
    }

    return controller_interface::return_type::OK;
}

} // namespace hqp_controller



PLUGINLIB_EXPORT_CLASS(
    hqp_controller::HQPController,
    controller_interface::ControllerInterface
)