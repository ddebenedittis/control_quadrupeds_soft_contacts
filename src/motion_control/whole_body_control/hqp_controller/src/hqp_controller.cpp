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
/*                                HQPPUBLISHER                                */
/* ========================================================================== */

HQPPublisher::HQPPublisher()
: Node("HQP_publisher")
{
    joints_accelerations_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/optimal_joints_accelerations", 1);

    torques_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/optimal_torques", 1);

    forces_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/optimal_forces", 1);

    deformations_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/optimal_deformations", 1);

    feet_positions_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/feet_positions", 1);

    feet_velocities_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/feet_velocities", 1);
}

void HQPPublisher::publish_all(
    const Eigen::VectorXd& joints_accelerations, const Eigen::VectorXd& torques,
    const Eigen::VectorXd& forces, const Eigen::VectorXd& deformations,
    const Eigen::VectorXd& feet_positions, const Eigen::VectorXd& feet_velocities)
{
    // Convert the joints accelerations to a ROS message and publish it
    auto joints_accelerations_message = std_msgs::msg::Float64MultiArray();
    joints_accelerations_message.data = std::vector<double>(joints_accelerations.data(), joints_accelerations.data() + joints_accelerations.size());
    joints_accelerations_publisher_->publish(joints_accelerations_message);

    // Same for the optimal torques
    auto torques_message = std_msgs::msg::Float64MultiArray();
    torques_message.data = std::vector<double>(torques.data(), torques.data() + torques.size());
    torques_publisher_->publish(torques_message);

    // Same for the optimal contact forces
    auto forces_message = std_msgs::msg::Float64MultiArray();
    forces_message.data = std::vector<double>(forces.data(), forces.data() + forces.size());
    forces_publisher_->publish(forces_message);

    // Same for the optimal feet deformations
    auto deformations_message = std_msgs::msg::Float64MultiArray();
    deformations_message.data = std::vector<double>(deformations.data(), deformations.data() + deformations.size());
    deformations_publisher_->publish(deformations_message);

    auto feet_positions_message = std_msgs::msg::Float64MultiArray();
    feet_positions_message.data = std::vector<double>(feet_positions.data(), feet_positions.data() + feet_positions.size());
    feet_positions_publisher_->publish(feet_positions_message);

    auto feet_velocities_message = std_msgs::msg::Float64MultiArray();
    feet_velocities_message.data = std::vector<double>(feet_velocities.data(), feet_velocities.data() + feet_velocities.size());
    feet_velocities_publisher_->publish(feet_velocities_message);
}



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

        auto_declare<double>("initialization_time", double());

        auto_declare<std::vector<double>>("qi", std::vector<double>());

        auto_declare<double>("PD_proportional", double());
        auto_declare<double>("PD_derivative", double());

        auto_declare<bool>("logging", bool());

        auto_declare<std::string>("contact_constraint_type", std::string());
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
        auto_declare<std::vector<double>>("kc_v", std::vector<double>());

        auto_declare<double>("regularization", double());
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

CallbackReturn HQPController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
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


    init_time_ = get_node()->get_parameter("initialization_time").as_double();


    /* ====================================================================== */

    auto qi = get_node()->get_parameter("qi").as_double_array();
    if (qi.size() != 12) {
        RCLCPP_ERROR(get_node()->get_logger(),"'qi' does not have 12 elements");
        return CallbackReturn::ERROR;
    }
    qi_ = Eigen::VectorXd::Map(qi.data(), qi.size());

    PD_proportional_ = get_node()->get_parameter("PD_proportional").as_double();
    PD_derivative_ = get_node()->get_parameter("PD_derivative").as_double();


    /* ====================================================================== */

    logging_ = get_node()->get_parameter("logging").as_bool();


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


    if (get_node()->get_parameter("contact_constraint_type").as_string().empty()) {
        RCLCPP_ERROR(get_node()->get_logger(),"'contact_constraint_type' parameter is empty");
        return CallbackReturn::ERROR;
    }
    wbc.set_contact_constraint_type(get_node()->get_parameter("contact_constraint_type").as_string());
    int def_size = wbc.get_def_size();

    if (get_node()->get_parameter("tau_max").as_double() <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'tau_max' parameter is <= 0");
        return CallbackReturn::ERROR;
    }
    wbc.set_tau_max(get_node()->get_parameter("tau_max").as_double());
    
    if (get_node()->get_parameter("mu").as_double() <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'mu' parameter is <= 0");
        return CallbackReturn::ERROR;
    }
    wbc.set_mu(get_node()->get_parameter("mu").as_double());
    
    if (get_node()->get_parameter("Fn_max").as_double() <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'Fn_max' parameter is <= 0");
        return CallbackReturn::ERROR;
    }
    wbc.set_Fn_max(get_node()->get_parameter("Fn_max").as_double());
    
    if (get_node()->get_parameter("Fn_min").as_double() <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'Fn_min' parameter is <= 0");
        return CallbackReturn::ERROR;
    }
    wbc.set_Fn_min(get_node()->get_parameter("Fn_min").as_double());


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

    if (def_size == 1) {
        RCLCPP_INFO(get_node()->get_logger(),"Only the last element of 'kp_terr' and 'kd_terr' parameter will be used");
    }

    if (get_node()->get_parameter("kc_v").as_double_array().size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),"'kc_v' parameter does not have three elements");
        return CallbackReturn::ERROR;
    }
    wbc.set_kc_v(Eigen::Vector3d::Map(get_node()->get_parameter("kc_v").as_double_array().data()));

    if (get_node()->get_parameter("regularization").as_double() < 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'regularization' parameter must be >= 0");
        return CallbackReturn::ERROR;
    }
    wbc.set_regularization(get_node()->get_parameter("regularization").as_double());


    /* ====================================================================== */

    if (use_estimator == false) {
        // Use the simulated data of Gazebo.

        joint_state_subscription_ = get_node()->create_subscription<gazebo_msgs::msg::LinkStates>(
            "/gazebo/link_states", QUEUE_SIZE,
            [this](const gazebo_msgs::msg::LinkStates::SharedPtr msg) -> void
            {
                int base_id = -1;

                for (std::size_t i=0; i<msg->name.size(); i++) {
                    if (msg->name[i].size() >= 4) {
                        if (msg->name[i].find("base") != std::string::npos) {
                            base_id = i;
                            break;
                        }
                    }
                }

                if (base_id == -1) {
                    RCLCPP_ERROR(get_node()->get_logger(),"Can't find a link name which contains 'base'");
                }

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


    /* ====================================================================== */

    if (logging_ == true) {
        logger_ = std::make_shared<HQPPublisher>();
    }


    return CallbackReturn::SUCCESS;
}


/* =============================== On_activate ============================== */

CallbackReturn HQPController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}


/* ============================== On_deactivate ============================= */

CallbackReturn HQPController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}


/* ================================= Update ================================= */

controller_interface::return_type HQPController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/
) {
    double time_double = static_cast<double>(time.seconds()) + static_cast<double>(time.nanoseconds()) * std::pow(10, -9);

    if (des_gen_pose_.contact_feet_names.size() + des_gen_pose_.feet_pos.size()/3 != 4) {
        // The planner is not publishing messages yet. Interpolate from q0 to qi and than wait.

        Eigen::VectorXd q = std::min(1., time_double / init_time_) * qi_;

        // PD for the state estimator initialization
        for (uint i=0; i<joint_names_.size(); i++) {
            command_interfaces_[i].set_value(
                + PD_proportional_ * (q[i] - state_interfaces_[2*i].get_value())
                + PD_derivative_ * (- state_interfaces_[2*i+1].get_value())
            );
        }
    } else {
        // WBC

        for (uint i=0; i<joint_names_.size(); i++) {
            q_(i+7) = state_interfaces_[2*i].get_value();
            v_(i+6) = state_interfaces_[2*i+1].get_value();
        }

        wbc::GeneralizedPose des_gen_pose_copy = des_gen_pose_;
        
        wbc.step(q_, v_, des_gen_pose_copy);

        tau_ = wbc.get_tau_opt();

        // Send effort command
        for (uint i=0; i<joint_names_.size(); i++) {
            command_interfaces_[i].set_value(tau_(i));
        }

        if (logging_ == true) {
            logger_->publish_all(
                wbc.get_v_dot_opt(), tau_,
                wbc.get_f_c_opt(), wbc.get_d_des_opt(),
                wbc.get_feet_positions(), wbc.get_feet_velocities(v_));
        }
    }

    return controller_interface::return_type::OK;
}

} // namespace hqp_controller



PLUGINLIB_EXPORT_CLASS(
    hqp_controller::HQPController,
    controller_interface::ControllerInterface
)