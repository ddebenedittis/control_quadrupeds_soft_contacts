#include "hqp_controller/hqp_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"



#define QUEUE_SIZE 1



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

        auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());

        auto_declare<bool>("use_estimator", bool());

        auto_declare<int>("initialization_counter", int());

        auto_declare<double>("PD_proportional", double());
        auto_declare<double>("PD_derivative", double());

        auto_declare<double>("tau_max", double());
        auto_declare<double>("mu", double());
        auto_declare<double>("Fn_max", double());
        auto_declare<double>("Fn_min", double());

        auto_declare<std::vector<double>>("kp_b_pos", std::vector<double>());
        auto_declare<std::vector<double>>("kd_b_pos", std::vector<double>());
        auto_declare<std::vector<double>>("kp_b_ang", std::vector<double>());
        auto_declare<std::vector<double>>("kd_b_ang", std::vector<double>());
        auto_declare<std::vector<double>>("kp_s_pos", std::vector<double>());
        auto_declare<std::vector<double>>("kd_s_pos", std::vector<double>());
        auto_declare<std::vector<double>>("kp_terr", std::vector<double>());
        auto_declare<std::vector<double>>("kd_terr", std::vector<double>());
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


    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(),"'joints' is empty");
        return CallbackReturn::ERROR;
    }


    bool use_estimator = get_node()->get_parameter("use_estimator").as_bool();


    counter_ = get_node()->get_parameter("initialization_counter").as_int();


    /* ====================================================================== */

    PD_proportional_ = get_node()->get_parameter("PD_proportional").as_double();
    PD_derivative_ = get_node()->get_parameter("PD_derivative").as_double();


    /* ====================================================================== */

    wbc = wbc::WholeBodyController(robot_name, dt);

    q_.resize(wbc.get_nv() + 1);
    q_(6) = 1;
    v_.resize(wbc.get_nv());

    des_gen_pose_.feet_pos.resize(3);
    des_gen_pose_.feet_vel.resize(3);
    des_gen_pose_.feet_acc.resize(3);
    des_gen_pose_.feet_pos.resize(0);
    des_gen_pose_.feet_vel.resize(0);
    des_gen_pose_.feet_acc.resize(0);

    wbc.set_tau_max(get_node()->get_parameter("tau_max").as_double());
    if (get_node()->get_parameter("tau_max").as_double() <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'tau_max' parameter is <= 0");
        return CallbackReturn::ERROR;
    }
    
    wbc.set_mu(get_node()->get_parameter("mu").as_double());
    if (get_node()->get_parameter("mu").as_double() <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'mu' parameter is <= 0");
        return CallbackReturn::ERROR;
    }
    
    wbc.set_Fn_max(get_node()->get_parameter("Fn_max").as_double());
    if (get_node()->get_parameter("Fn_max").as_double() <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'Fn_max' parameter is <= 0");
        return CallbackReturn::ERROR;
    }
    
    wbc.set_Fn_min(get_node()->get_parameter("Fn_min").as_double());
    if (get_node()->get_parameter("Fn_min").as_double() <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'Fn_min' parameter is <= 0");
        return CallbackReturn::ERROR;
    }


    if (get_node()->get_parameter("kp_b_pos").as_double_array().size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),"'kp_b_pos' parameter does not have three elements");
        return CallbackReturn::ERROR;
    }
    wbc.set_kp_b_pos(Eigen::Vector3d::Map(get_node()->get_parameter("kp_b_pos").as_double_array().data()));

    if (get_node()->get_parameter("kd_b_pos").as_double_array().size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),"'kd_b_pos' parameter does not have three elements");
        return CallbackReturn::ERROR;
    }
    wbc.set_kd_b_pos(Eigen::Vector3d::Map(get_node()->get_parameter("kd_b_pos").as_double_array().data()));


    if (get_node()->get_parameter("kp_b_ang").as_double_array().size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),"'kp_b_ang' parameter does not have three elements");
        return CallbackReturn::ERROR;
    }
    wbc.set_kp_b_ang(Eigen::Vector3d::Map(get_node()->get_parameter("kp_b_ang").as_double_array().data()));

    if (get_node()->get_parameter("kd_b_ang").as_double_array().size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),"'kd_b_ang' parameter does not have three elements");
        return CallbackReturn::ERROR;
    }
    wbc.set_kd_b_ang(Eigen::Vector3d::Map(get_node()->get_parameter("kd_b_ang").as_double_array().data()));


    if (get_node()->get_parameter("kp_s_pos").as_double_array().size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),"'kp_s_pos' parameter does not have three elements");
        return CallbackReturn::ERROR;
    }
    wbc.set_kp_s_pos(Eigen::Vector3d::Map(get_node()->get_parameter("kp_s_pos").as_double_array().data()));

    if (get_node()->get_parameter("kd_s_pos").as_double_array().size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),"'kd_s_pos' parameter does not have three elements");
        return CallbackReturn::ERROR;
    }
    wbc.set_kd_s_pos(Eigen::Vector3d::Map(get_node()->get_parameter("kd_s_pos").as_double_array().data()));


    if (get_node()->get_parameter("kp_terr").as_double_array().size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),"'kp_terr' parameter does not have three elements");
        return CallbackReturn::ERROR;
    }
    wbc.set_kp_terr(Eigen::Vector3d::Map(get_node()->get_parameter("kp_terr").as_double_array().data()));

    if (get_node()->get_parameter("kd_terr").as_double_array().size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),"'kd_terr' parameter does not have three elements");
        return CallbackReturn::ERROR;
    }
    wbc.set_kd_terr(Eigen::Vector3d::Map(get_node()->get_parameter("kd_terr").as_double_array().data()));


    /* ====================================================================== */

    if (use_estimator == false) {
        // Use the simulated data of Gazebo.

        joint_state_subscription_ = get_node()->create_subscription<gazebo_msgs::msg::LinkStates>(
            "/gazebo/link_states", QUEUE_SIZE,
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
    } else {
        // Use the state estimator measurements.

        estimated_pose_subscription_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
            "/state_estimator/pose", QUEUE_SIZE,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) -> void
            {
                q_.head(7) << msg->position.x, msg->position.y, msg->position.z,
                            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
            }
        );

        estimated_twist_subscription_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "/state_estimator/twist", QUEUE_SIZE,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
            {
                v_.head(6) << msg->linear.x, msg->linear.y, msg->linear.z,
                            msg->angular.x, msg->angular.y, msg->angular.z;
            }
        );
    }
    

    desired_generalized_pose_subscription_ = get_node()->create_subscription<generalized_pose_msgs::msg::GeneralizedPose>(
        "/robot/desired_generalized_pose", QUEUE_SIZE,
        [this](const generalized_pose_msgs::msg::GeneralizedPose::SharedPtr msg) -> void
        {
            des_gen_pose_.base_acc << msg->base_acc.x, msg->base_acc.y, msg->base_acc.z;
            des_gen_pose_.base_vel << msg->base_vel.x, msg->base_vel.y, msg->base_vel.z;
            des_gen_pose_.base_pos << msg->base_pos.x, msg->base_pos.y, msg->base_pos.z;

            des_gen_pose_.base_angvel << msg->base_angvel.x, msg->base_angvel.y, msg->base_angvel.z;
            des_gen_pose_.base_quat << msg->base_quat.x, msg->base_quat.y, msg->base_quat.z, msg->base_quat.w;

            des_gen_pose_.feet_acc = Eigen::VectorXd::Map(msg->feet_acc.data(), msg->feet_acc.size());
            des_gen_pose_.feet_vel = Eigen::VectorXd::Map(msg->feet_vel.data(), msg->feet_vel.size());
            des_gen_pose_.feet_pos = Eigen::VectorXd::Map(msg->feet_pos.data(), msg->feet_pos.size());

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
    const rclcpp::Time& timee, const rclcpp::Duration& period
) {
    if (counter_ > 0) {
        counter_--;

        // ! ATTENTION: deactivate the PD for SOLO, and activate it for ANYmal.
        // TODO: understand what condition is q0 for solo
        // PD for the state estimator initialization
        for (uint i=0; i<joint_names_.size(); i++) {
            command_interfaces_[i].set_value(
                - PD_proportional_ * state_interfaces_[2*i].get_value()
                - PD_derivative_ * state_interfaces_[2*i+1].get_value()
            );
        }
    } else {
        // WBC

        for (uint i=0; i<joint_names_.size(); i++) {
            q_(i+7) = state_interfaces_[2*i].get_value();
            v_(i+6) = state_interfaces_[2*i+1].get_value();
        }

        if (des_gen_pose_.contact_feet_names.size() + des_gen_pose_.feet_pos.size()/3 != 4) {
            return controller_interface::return_type::OK;
        }

        wbc::GeneralizedPose des_gen_pose_copy = des_gen_pose_;
        
        wbc.step(q_, v_, des_gen_pose_copy);

        tau_ = wbc.get_tau_opt();

        // Send effort command
        for (uint i=0; i<joint_names_.size(); i++) {
            command_interfaces_[i].set_value(tau_(i));
        }
    }

    return controller_interface::return_type::OK;
}

} // namespace hqp_controller



PLUGINLIB_EXPORT_CLASS(
    hqp_controller::HQPController,
    controller_interface::ControllerInterface
)