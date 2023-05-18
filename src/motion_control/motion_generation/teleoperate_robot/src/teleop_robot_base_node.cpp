#include "teleoperate_robot/teleop_robot_base_node.hpp"

#include "teleoperate_robot/quat_math.h"



namespace teleoperate_robot
{

TeleopRobotBase::TeleopRobotBase()
: TeleopRobot(),
  l_twist_{0.0, 0.0, 0.0},
  a_twist_{0.0, 0.0, 0.0},
  l_scale_(1.0),
  a_scale_(1.0)
{
    this->declare_parameter("scale_angular", rclcpp::ParameterValue(1.0));
    this->declare_parameter("scale_linear", rclcpp::ParameterValue(1.0));
    this->declare_parameter("robot_name", rclcpp::ParameterValue("generic_robot_name"));
    this->get_parameter("scale_angular", a_scale_);
    this->get_parameter("scale_linear", l_scale_);
    this->get_parameter("robot_name", robot_name);

    pub_ = this->create_publisher<generalized_pose_msgs::msg::GeneralizedPose>(
        "motion_planner/desired_generalized_pose", 1);

    subscriber_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = subscriber_cb_group_;

    base_pose_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        "gazebo/model_states", 1, std::bind(&TeleopRobotBase::pose_callback, this, _1), sub_opt);

    print_instructions();
}


void TeleopRobotBase::print_instructions()
{
    puts(u8"\033[2J\033[1;1H"); // clears the terminal (only on linux)
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use the following keys (CASE SENSITIVE!) to move the robot base:");
    puts("w: tilt forward,          s: tilt backward");
    puts("d: tilt to the right,     a: tilt to the left");
    puts("q: tilt up,               e: tilt down");
    puts("---------------------------");
    puts("Use the following keys (CASE SENSITIVE!) to rotate the robot base:");
    puts("i: positive pitch,        k: negative pitch");
    puts("l: positive roll,         j: negative roll");
    puts("u: positive yaw,          o: negative yaw");

    puts("");
    puts("");
    puts("");
    puts("The commanded pose is:");
    printf("Position (x, y, z): %.2f, %.2f, %.2f\n", base_pos_des_[0], base_pos_des_[1], base_pos_des_[2]);
    printf("Orientation (qx, qy, qz, qw): %.3f, %.3f, %.3f, %.3f\n", base_quat_des_[0], base_quat_des_[1], base_quat_des_[2], base_quat_des_[3]);
}


bool TeleopRobotBase::process_key(const char c)
{
    l_twist_ = a_twist_ = {0.0, 0.0, 0.0};
    RCLCPP_DEBUG(this->get_logger(), "value: 0x%02X\n", c);

    switch(c) {
    // Linear movements of the base
    case KEYCODE_w:
        RCLCPP_DEBUG(this->get_logger(), "TILT FORWARD\n");
        l_twist_ = {1.0, 0.0, 0.0};
        return true;
    case KEYCODE_s:
        RCLCPP_DEBUG(this->get_logger(), "TILT BACKWARD\n");
        l_twist_ = {-1.0, 0.0, 0.0};
        return true;
    case KEYCODE_d:
        RCLCPP_DEBUG(this->get_logger(), "TILT TO THE RIGHT\n");
        l_twist_ = {0.0, -1.0, 0.0};
        return true;
    case KEYCODE_a:
        RCLCPP_DEBUG(this->get_logger(), "TILT TO THE LEFT\n");
        l_twist_ = {0.0, 1.0, 0.0};
        return true;
    case KEYCODE_q:
        RCLCPP_DEBUG(this->get_logger(), "TILT UP\n");
        l_twist_ = {0.0, 0.0, 1.0};
        return true;
    case KEYCODE_e:
        RCLCPP_DEBUG(this->get_logger(), "TILT DOWN\n");
        l_twist_ = {0.0, 0.0, -1.0};
        return true;

    // Angular movements of the base
    case KEYCODE_l:
        RCLCPP_DEBUG(this->get_logger(), "POSITIVE ROLL\n");
        a_twist_ = {1.0, 0.0, 0.0};
        return true;
    case KEYCODE_j:
        RCLCPP_DEBUG(this->get_logger(), "NEGATIVE ROLL\n");
        a_twist_ = {-1.0, 0.0, 0.0};
        return true;
    case KEYCODE_i:
        RCLCPP_DEBUG(this->get_logger(), "NEGATIVE PITCH\n");
        a_twist_ = {0.0, -1.0, 0.0};
        return true;
    case KEYCODE_k:
        RCLCPP_DEBUG(this->get_logger(), "POSITIVE PITCH\n");
        a_twist_ = {0.0, 1.0, 0.0};
        return true;
    case KEYCODE_u:
        RCLCPP_DEBUG(this->get_logger(), "POSITIVE YAW\n");
        a_twist_ = {0.0, 0.0, 1.0};
        return true;
    case KEYCODE_o:
        RCLCPP_DEBUG(this->get_logger(), "NEGATIVE YAW\n");
        a_twist_ = {0.0, 0.0, -1.0};
        return true;
    }

    return false;
}


bool TeleopRobotBase::initialize_message()
{
    msg_.base_acc.x = 0;
    msg_.base_acc.y = 0;
    msg_.base_acc.z = 0;

    msg_.base_vel.x = 0;
    msg_.base_vel.y = 0;
    msg_.base_vel.z = 0;

    msg_.base_pos.x = base_pos_des_[0];
    msg_.base_pos.y = base_pos_des_[1];
    msg_.base_pos.z = base_pos_des_[2];

    msg_.base_angvel.x = 0;
    msg_.base_angvel.y = 0;
    msg_.base_angvel.z = 0;

    msg_.base_quat.x = base_quat_des_[0];
    msg_.base_quat.y = base_quat_des_[1];
    msg_.base_quat.z = base_quat_des_[2];
    msg_.base_quat.w = base_quat_des_[3];

    msg_.contact_feet = {"LF", "RF", "LH", "RH"};

    if (quat_norm(base_quat_des_) > 0.9) {
        base_pose_sub_.reset();
        return true;
    } else {
        return false;
    }
}


void TeleopRobotBase::update_message()
{
    // Update the desired base position and orientation using the commanded linear and angular twist.

    // Update the desired base position
    for(int i=0; i<static_cast<int>(base_pos_des_.size()); i++) {
        base_pos_des_[i] = base_pos_des_[i] + l_twist_[i] * l_scale_ * 0.01;
    }

    // Integrate the orientation with the angular velocity in body frame received by the user input.
    base_quat_des_ = quat_int(base_quat_des_, a_twist_, a_scale_ * 0.01);


    // Set the message fields.

    msg_.base_acc.x = 0;
    msg_.base_acc.y = 0;
    msg_.base_acc.z = 0;

    msg_.base_vel.x = 0;
    msg_.base_vel.y = 0;
    msg_.base_vel.z = 0;

    msg_.base_pos.x = base_pos_des_[0];
    msg_.base_pos.y = base_pos_des_[1];
    msg_.base_pos.z = base_pos_des_[2];

    msg_.base_angvel.x = 0;
    msg_.base_angvel.y = 0;
    msg_.base_angvel.z = 0;

    msg_.base_quat.x = base_quat_des_[0];
    msg_.base_quat.y = base_quat_des_[1];
    msg_.base_quat.z = base_quat_des_[2];
    msg_.base_quat.w = base_quat_des_[3];

    msg_.contact_feet = {"LF", "RF", "LH", "RH"};
}

/* ============================== Pose_callback ============================= */

void TeleopRobotBase::pose_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
    int base_id = -1;

    for (int i = 0; i < static_cast<int>(msg->name.size()); i++) {
        if (msg->name[i] == robot_name) {
            base_id = i;
            break;
        }
    }

    base_pos_des_ = {
        msg->pose[base_id].position.x,
        msg->pose[base_id].position.y,
        msg->pose[base_id].position.z
    };

    base_quat_des_[0] = msg->pose[base_id].orientation.x;
    base_quat_des_[1] = msg->pose[base_id].orientation.y;
    base_quat_des_[2] = msg->pose[base_id].orientation.z;
    base_quat_des_[3] = msg->pose[base_id].orientation.w;
}

} // teleoperate_robot



/* ========================================================================== */
/*                                    MAIN                                    */
/* ========================================================================== */

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // TeleopRobot teleop_robot;
    auto teleop_robot_base = std::make_shared<teleoperate_robot::TeleopRobotBase>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(teleop_robot_base);
    executor.spin();

    teleop_robot_base->shutdown();

    rclcpp::shutdown();

    return 0;
}