#include "static_walk_planner/static_walk_planner_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <utility>



namespace static_walk_planner {

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;



/* ========================================================================== */
/*                                SWPCONTROLLER                               */
/* ========================================================================== */

SWPController::SWPController()
: controller_interface::ControllerInterface(),
  planner_() {}


/* ================================= On_init ================================ */

CallbackReturn SWPController::on_init()
{
    try {
        auto_declare<double>("sample_time", double());
        auto_declare<double>("init_phase", double());
        auto_declare<std::vector<std::string>>("all_feet_names", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("gait_pattern", std::vector<std::string>());
        auto_declare<double>("cycle_duration", double());
        auto_declare<double>("step_duty_factor", double());
        auto_declare<double>("step_length", double());
        auto_declare<double>("step_height", double());
        auto_declare<double>("desired_foot_penetration", double());
        auto_declare<double>("desired_base_height", double());
        auto_declare<double>("initial_base_height", double());
        auto_declare<std::vector<double>>("initial_position", std::vector<double>());
        auto_declare<std::vector<double>>("leg_position", std::vector<double>());
        auto_declare<std::vector<double>>("base_oscillation", std::vector<double>());
    }
    catch(const std::exception& e) {
        fprintf(stderr,"Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}


/* ===================== Command_interface_configuration ==================== */

InterfaceConfiguration SWPController::command_interface_configuration() const
{
    InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = interface_configuration_type::NONE;

    return command_interfaces_config;
}


/* ====================== State_interface_configuration ===================== */

InterfaceConfiguration SWPController::state_interface_configuration() const
{
    InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = interface_configuration_type::NONE;

    return state_interfaces_config;
}


/* ============================== On_configure ============================== */

CallbackReturn SWPController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
    planner_.dt_ = get_node()->get_parameter("sample_time").as_double();
    if (planner_.dt_ <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'sample_time' parameter is <= 0.");
        return CallbackReturn::ERROR;
    }

    planner_.init_phase_ = get_node()->get_parameter("init_phase").as_double();
    if (planner_.init_phase_ <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'init_phase' parameter is <= 0.");
        return CallbackReturn::ERROR;
    }

    planner_.all_feet_names_ = get_node()->get_parameter("all_feet_names").as_string_array();
    if (planner_.all_feet_names_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(),"'all_feet_names' parameter is empty");
        return CallbackReturn::ERROR;
    }

    planner_.gait_pattern_ = get_node()->get_parameter("gait_pattern").as_string_array();
    if (planner_.gait_pattern_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(),"'gait_pattern' parameter is empty.");
        return CallbackReturn::ERROR;
    }

    planner_.cycle_duration_ = get_node()->get_parameter("cycle_duration").as_double();
    if (planner_.cycle_duration_ <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'cycle_duration' parameter is <= 0.");
        return CallbackReturn::ERROR;
    }

    planner_.step_duty_factor_ = get_node()->get_parameter("step_duty_factor").as_double();
    if (planner_.step_duty_factor_ <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'step_duty_factor' parameter is <= 0.");
        return CallbackReturn::ERROR;
    }

    planner_.step_length_ = get_node()->get_parameter("step_length").as_double();
    if (planner_.step_length_ <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'step_length' parameter is <= 0.");
        return CallbackReturn::ERROR;
    }

    planner_.step_height_ = get_node()->get_parameter("step_height").as_double();
    if (planner_.step_height_ <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'step_height' parameter is <= 0.");
        return CallbackReturn::ERROR;
    }

    planner_.desired_foot_penetration_ = get_node()->get_parameter("desired_foot_penetration").as_double();
    if (planner_.desired_foot_penetration_ <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'desired_foot_penetration' parameter is <= 0.");
        return CallbackReturn::ERROR;
    }

    planner_.h_base_des_ = get_node()->get_parameter("desired_base_height").as_double();
    if (planner_.h_base_des_ <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'desired_base_height' parameter is <= 0.");
        return CallbackReturn::ERROR;
    }

    planner_.h_base_init = get_node()->get_parameter("initial_base_height").as_double();
    if (planner_.h_base_init <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'initial_base_height' parameter is <= 0.");
        return CallbackReturn::ERROR;
    }

    planner_.init_com_position_ = std::make_pair(get_node()->get_parameter("initial_position").as_double_array()[0],
                                                 get_node()->get_parameter("initial_position").as_double_array()[1]);
    if (get_node()->get_parameter("initial_position").as_double_array().size() != 2) {
        RCLCPP_ERROR(get_node()->get_logger(),"'initial_position' does not have two elements.");
        return CallbackReturn::ERROR;
    }

    planner_.abs_leg_pos_ = std::make_pair(get_node()->get_parameter("leg_position").as_double_array()[0],
                                           get_node()->get_parameter("leg_position").as_double_array()[1]);
    if (get_node()->get_parameter("leg_position").as_double_array().size() != 2) {
        RCLCPP_ERROR(get_node()->get_logger(),"'leg_position' does not have two elements.");
        return CallbackReturn::ERROR;
    }

    planner_.base_osc_ = std::make_pair(get_node()->get_parameter("base_oscillation").as_double_array()[0],
                                        get_node()->get_parameter("base_oscillation").as_double_array()[1]);
    if (get_node()->get_parameter("base_oscillation").as_double_array().size() != 2) {
        RCLCPP_ERROR(get_node()->get_logger(),"'base_oscillation' does not have two elements.");
        return CallbackReturn::ERROR;
    }

    /* ====================================================================== */

    gen_pose_publisher_ = get_node()->create_publisher<generalized_pose_msgs::msg::GeneralizedPose>(
        "/robot/desired_generalized_pose", rclcpp::SystemDefaultsQoS()
    );


    return CallbackReturn::SUCCESS;
}


/* =============================== On_activate ============================== */

CallbackReturn SWPController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    return CallbackReturn::SUCCESS;
}


/* ============================== On_deactivate ============================= */

CallbackReturn SWPController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    return CallbackReturn::SUCCESS;
}


/* ================================= Update ================================= */

controller_interface::return_type SWPController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    planner_.step(gen_pose_);

    publish_gen_pose();

    return controller_interface::return_type::OK;
}


/* ============================ Publish_gen_pose ============================ */

void SWPController::publish_gen_pose()
{
    generalized_pose_msgs::msg::GeneralizedPose msg;

    msg.base_acc.x = gen_pose_.base_acc[0];
    msg.base_acc.y = gen_pose_.base_acc[1];
    msg.base_acc.z = gen_pose_.base_acc[2];

    msg.base_vel.x = gen_pose_.base_vel[0];
    msg.base_vel.y = gen_pose_.base_vel[1];
    msg.base_vel.z = gen_pose_.base_vel[2];

    msg.base_pos.x = gen_pose_.base_pos[0];
    msg.base_pos.y = gen_pose_.base_pos[1];
    msg.base_pos.z = gen_pose_.base_pos[2];

    msg.base_angvel.x = gen_pose_.base_angvel[0];
    msg.base_angvel.y = gen_pose_.base_angvel[1];
    msg.base_angvel.z = gen_pose_.base_angvel[2];

    msg.base_quat.x = gen_pose_.base_quat[0];
    msg.base_quat.y = gen_pose_.base_quat[1];
    msg.base_quat.z = gen_pose_.base_quat[2];
    msg.base_quat.w = gen_pose_.base_quat[3];

    msg.feet_acc = std::vector<double>(gen_pose_.feet_acc.data(), gen_pose_.feet_acc.data() + gen_pose_.feet_acc.size());
    msg.feet_vel = std::vector<double>(gen_pose_.feet_vel.data(), gen_pose_.feet_vel.data() + gen_pose_.feet_vel.size());
    msg.feet_pos = std::vector<double>(gen_pose_.feet_pos.data(), gen_pose_.feet_pos.data() + gen_pose_.feet_pos.size());
 
    msg.contact_feet = gen_pose_.contact_feet_names;
    for (auto& t : msg.contact_feet) {
        t += "_FOOT";
    }


    gen_pose_publisher_.get()->publish(msg);
}

} // namespace static_walk_planner



PLUGINLIB_EXPORT_CLASS(
    static_walk_planner::SWPController,
    controller_interface::ControllerInterface)