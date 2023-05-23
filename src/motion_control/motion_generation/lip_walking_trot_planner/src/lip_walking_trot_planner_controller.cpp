#include "lip_walking_trot_planner/lip_walking_trot_planner_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"



namespace lip_walking_trot_planner {

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

using namespace Eigen;



Quaterniond quat_mult(const Quaterniond& q1, const Quaterniond& q2)
{
    double x1 = q1.x();
    double y1 = q1.y();
    double z1 = q1.z();
    double w1 = q1.w();

    double x2 = q2.x();
    double y2 = q2.y();
    double z2 = q2.z();
    double w2 = q2.w();

    double x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    double y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
    double z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;
    double w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;

    return {w, x, y, z};
}

void quat_rot(const Quaterniond& quat, Vector3d& vec)
{
    Quaterniond vec_quat = Quaterniond(0, vec[0], vec[1], vec[2]);

    Quaternion vec_out = quat_mult(quat_mult(quat, vec_quat), quat.conjugate());

    vec[0] = vec_out.x();
    vec[1] = vec_out.y();
    vec[2] = vec_out.z();
}


Quaterniond compute_quaternion_from_euler_angles(double roll, double pitch, double yaw)
{
    double q_x = std::sin(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) - std::cos(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
    double q_y = std::cos(roll/2) * std::sin(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::cos(pitch/2) * std::sin(yaw/2);
    double q_z = std::cos(roll/2) * std::cos(pitch/2) * std::sin(yaw/2) - std::sin(roll/2) * std::sin(pitch/2) * std::cos(yaw/2);
    double q_w = std::cos(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);

    return {q_w, q_x, q_y, q_z};
}


/* ================================= On_init ================================ */

CallbackReturn LIPController::on_init()
{
    try {
        auto_declare<double>("zero_time", double());
        auto_declare<double>("init_time", double());

        auto_declare<double>("sample_time", double());
        auto_declare<double>("step_duration", double());
        auto_declare<double>("step_height", double());

        auto_declare<int>("acc_filter_order", int());
        auto_declare<double>("acc_filter_beta", double());
    }
    catch(const std::exception& e) {
        fprintf(stderr,"Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}


/* ===================== Command_interface_configuration ==================== */

InterfaceConfiguration LIPController::command_interface_configuration() const
{
    InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = interface_configuration_type::NONE;

    return command_interfaces_config;
}


/* ====================== State_interface_configuration ===================== */

InterfaceConfiguration LIPController::state_interface_configuration() const
{
    InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = interface_configuration_type::NONE;

    return state_interfaces_config;
}


/* ============================== On_configure ============================== */

CallbackReturn LIPController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    zero_time_ = get_node()->get_parameter("zero_time").as_double();
    if (zero_time_ < 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'zero_time' parameter is < 0.");
        return CallbackReturn::ERROR;
    }

    init_time_ = get_node()->get_parameter("init_time").as_double();
    if (init_time_ < 0) {
        RCLCPP_ERROR(get_node()->get_logger(),"'init_time' parameter is < 0.");
        return CallbackReturn::ERROR;
    }


    // sample_time_ = get_node()->get_parameter("sample_time").as_double();
    // if (sample_time_ < 0) {
    //     RCLCPP_ERROR(get_node()->get_logger(),"'sample_time' parameter is < 0.");
    //     return CallbackReturn::ERROR;
    // }

    // ste = get_node()->get_parameter("init_time").as_double();
    // if (init_time_ < 0) {
    //     RCLCPP_ERROR(get_node()->get_logger(),"'init_time' parameter is < 0.");
    //     return CallbackReturn::ERROR;
    // }

    // step_height = get_node()->get_parameter("init_time").as_double();
    // if (init_time_ < 0) {
    //     RCLCPP_ERROR(get_node()->get_logger(),"'init_time' parameter is < 0.");
    //     return CallbackReturn::ERROR;
    // }


    /* ============================= Subscribers ============================ */

    imu_subscription_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_sensor_broadcaster/imu", 1,
        [this](const sensor_msgs::msg::Imu& msg) -> void
        {
            auto acc = msg.linear_acceleration;

            a_ << acc.x, acc.y, acc.z;
        }
    );

    link_states_subscription_ = get_node()->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/gazebo/link_states", 1,
        [this](const gazebo_msgs::msg::LinkStates& msg) -> void
        {
            // The index of the base must be found by searching which link contains "base" in its name.
            int base_id = -1;

            for (int i = 0; i < static_cast<int>(msg.name.size()); i++) {
                if (msg.name[i].find("base") != std::string::npos) {
                    base_id = i;
                    break;
                }
            }
        
            // /gazebo/link_states returns the pose and the twist in the inertial or world frame.
        
            auto pos = msg.pose[base_id].position;
            auto quat = msg.pose[base_id].orientation;

            auto vel_lin = msg.twist[base_id].linear;
            auto vel_ang = msg.twist[base_id].angular;

            q_ << pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w;
            v_ << vel_lin.x, vel_lin.y, vel_lin.z, vel_ang.x, vel_ang.y, vel_ang.z;
        }
    );
    
    terrain_penetrations_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "state_estimator/terrain_penetration", 1,
        [this](const std_msgs::msg::Float64MultiArray& msg) -> void
        {
            terrain_penetrations_ << msg.data[0], msg.data[1], msg.data[2], msg.data[3];
        }
    );

    terrain_plane_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "state_estimator/terrain_plane", 1,
        [this](const std_msgs::msg::Float64MultiArray& msg) -> void
        {
            plane_coeffs_ << msg.data[0], msg.data[1], msg.data[2];
        }
    );
    
    simple_velocity_command_subscription_ = get_node()->create_subscription<velocity_command_msgs::msg::SimpleVelocityCommand>(
        "/motion_generator/simple_velocity_command", 1,
        [this](const velocity_command_msgs::msg::SimpleVelocityCommand& msg) -> void
        {
            velocity_forward_ = msg.velocity_forward;
            velocity_lateral_ = msg.velocity_lateral;
            yaw_rate_ = msg.yaw_rate;
        }
    );

    /* ============================= Publishers ============================= */

    gen_pose_publisher_ = get_node()->create_publisher<generalized_pose_msgs::msg::GeneralizedPose>(
        "/motion_planner/desired_generalized_pose", rclcpp::SystemDefaultsQoS()
    );
    
    return CallbackReturn::SUCCESS;
}


/* =============================== On_activate ============================== */

CallbackReturn LIPController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}


/* ============================== On_deactivate ============================= */

CallbackReturn LIPController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}


/* ================================= Update ================================= */

controller_interface::return_type LIPController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    double time_double = static_cast<double>(time.seconds()) + static_cast<double>(time.nanoseconds()) * std::pow(10, -9);

    if (time_double > init_time_ + zero_time_) {
        Quaterniond quat_conj = Quaterniond(
            q_[6], q_[3], q_[4], q_[5]
        );

        Vector3d g = {0, 0, - 9.81};

        Vector3d a_b_meas_body = a_;
        quat_rot(quat_conj, a_b_meas_body);
        a_b_meas_body += g;
        // Vector3d a_b_meas_body = quat_conj * a_ + g;

        Vector3d a_b = - filter_.filter(a_b_meas_body, planner_.get_sample_time());
        
        gen_pose_ = planner_.update(
            q_.head(3), v_.head(3), a_b,
            (Vector2d() << velocity_forward_, velocity_lateral_).finished(), yaw_rate_
        );
    } else if (time_double > zero_time_) {
        // Interpolate between the initial position and the starting position of the trot.

        Vector3d base_pos, base_vel, base_acc;

        Vector3d end_pos = init_pos_;
        end_pos[2] = planner_.get_height_com();

        std::tie(base_pos, base_vel, base_acc) = MotionPlanner::spline(
            init_pos_, 
            end_pos,
            (time_double - zero_time_) / init_time_,
            InterpolationMethod::Spline_5th
        );

        gen_pose_.base_acc = generalized_pose::Vector3(base_acc);
        gen_pose_.base_vel = generalized_pose::Vector3(base_vel);
        gen_pose_.base_pos = generalized_pose::Vector3(base_pos);

        gen_pose_.base_angvel = generalized_pose::Vector3(0, 0, 0);
        gen_pose_.base_quat = generalized_pose::Quaternion(
            0, 0, std::sin(planner_.get_dtheta()/2), std::cos(planner_.get_dtheta()/2)
        );
    } else {
        // Initialize the planner starting position and orientation.

        init_pos_ = q_.head(3);

        double dtheta = std::atan2(
            2 * (q_[3]*q_[2] + q_[0]*q_[1]), 
            1 - 2 * (q_[1]*q_[1] +  q_[2]*q_[2])
        );

        planner_.update_initial_conditions(init_pos_, dtheta);

        return controller_interface::return_type::OK;
    }

    // correct_with_terrain_plane();

    gen_pose_publisher_->publish(gen_pose_.get_msg());

    return controller_interface::return_type::OK;
}


void LIPController::correct_with_terrain_plane()
{
    // Local terrain height
    double delta_h =   plane_coeffs_[0] * gen_pose_.base_pos.x
                     + plane_coeffs_[1] * gen_pose_.base_pos.y
                     + plane_coeffs_[2];

    // Shift the desired base pose and the desired feet positions.
    gen_pose_.base_pos.z += delta_h;
    
    for (int i = 0; i < static_cast<int>(gen_pose_.feet_pos.size()); i+=3) {
        gen_pose_.feet_pos[i] += delta_h;
    }

    // Align the desired base pose and pitch angles to the local terrain plane.
    Quaterniond quat = compute_quaternion_from_euler_angles(
          std::atan(plane_coeffs_[1]),
        - std::atan(plane_coeffs_[0]),
          planner_.get_dtheta()
    );

    gen_pose_.base_quat = generalized_pose::Quaternion(
        quat.x(), quat.y(), quat.z(), quat.w()
    );
}

} // namespace lip_walking_trot_planner



PLUGINLIB_EXPORT_CLASS(
    lip_walking_trot_planner::LIPController,
    controller_interface::ControllerInterface)