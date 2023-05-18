#include "teleoperate_robot/teleop_velocity_command_node.hpp"

#include "teleoperate_robot/quat_math.h"



namespace teleoperate_robot
{

TeleopSimpleVelocityCommand::TeleopSimpleVelocityCommand()
: TeleopRobot(),
  l_scale_(1.0),
  a_scale_(1.0)
{
    this->declare_parameter("scale_angular", rclcpp::ParameterValue(1.0));
    this->declare_parameter("scale_linear", rclcpp::ParameterValue(1.0));
    this->get_parameter("scale_angular", a_scale_);
    this->get_parameter("scale_linear", l_scale_);

    pub_ = this->create_publisher<velocity_command_msgs::msg::SimpleVelocityCommand>(
        "/motion_generator/simple_velocity_command", 1);

    print_instructions();
}


void TeleopSimpleVelocityCommand::print_instructions()
{
    puts(u8"\033[2J\033[1;1H"); // clears the terminal (only on linux)
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use the following keys (CASE SENSITIVE!) to move the robot base:");
    puts("w: move forward,          s: move backward");
    puts("d: move to the right,     a: move to the left");
    puts("---------------------------");
    puts("Use the following keys (CASE SENSITIVE!) to rotate the robot base:");
    puts("u: positive yaw,          o: negative yaw");
    puts("---------------------------");
    puts("R: Reset to zero");

    puts("");
    puts("");
    puts("");
    puts("The commanded velocity is:");
    printf("Forward velocity: %.2f\n", velocity_forward_);
    printf("Lateral velocity: %.2f\n", velocity_lateral_);
    printf("Yaw rate: %.2f\n", yaw_rate_);
}


bool TeleopSimpleVelocityCommand::process_key(const char c)
{
    RCLCPP_DEBUG(this->get_logger(), "value: 0x%02X\n", c);

    switch(c) {
    // Linear movements of the base
    case KEYCODE_w:
        RCLCPP_DEBUG(this->get_logger(), "MOVE FORWARD\n");
        velocity_forward_ += l_scale_ * 0.1;
        return true;
    case KEYCODE_s:
        RCLCPP_DEBUG(this->get_logger(), "MOVE BACKWARD\n");
        velocity_forward_ -= l_scale_ * 0.1;
        return true;
    case KEYCODE_d:
        RCLCPP_DEBUG(this->get_logger(), "MOVE TO THE RIGHT\n");
        velocity_lateral_ -= l_scale_ * 0.1;
        return true;
    case KEYCODE_a:
        RCLCPP_DEBUG(this->get_logger(), "MOVE TO THE LEFT\n");
        velocity_lateral_ += l_scale_ * 0.1;
        return true;

    // Angular movements of the base
    case KEYCODE_u:
        RCLCPP_DEBUG(this->get_logger(), "POSITIVE YAW\n");
        yaw_rate_ += a_scale_ * 0.1;
        return true;
    case KEYCODE_o:
        RCLCPP_DEBUG(this->get_logger(), "NEGATIVE YAW\n");
        yaw_rate_ -= a_scale_ * 0.1;
        return true;

    case KEYCODE_R:
        RCLCPP_DEBUG(this->get_logger(), "RESET\n");
        velocity_forward_ = 0;
        velocity_lateral_ = 0;
        yaw_rate_ = 0;
        return true;
    }

    return false;
}


void TeleopSimpleVelocityCommand::update_message()
{
    // Set the message fields.

    msg_.velocity_forward = velocity_forward_;
    msg_.velocity_lateral = velocity_lateral_;
    msg_.yaw_rate = yaw_rate_;
}

} // teleoperate_robot



/* ========================================================================== */
/*                                    MAIN                                    */
/* ========================================================================== */

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto teleop_simple_velocity_command = std::make_shared<teleoperate_robot::TeleopSimpleVelocityCommand>();

    rclcpp::spin(teleop_simple_velocity_command);

    teleop_simple_velocity_command->shutdown();

    rclcpp::shutdown();

    return 0;
}