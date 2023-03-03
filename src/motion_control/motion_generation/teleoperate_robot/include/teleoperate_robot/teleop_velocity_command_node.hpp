#pragma once

#include "teleoperate_robot/teleop_robot.tpp"

#include "velocity_command_msgs/msg/simple_velocity_command.hpp"



/* ========================================================================== */
/*                            CONSTANTS DEFINITION                            */
/* ========================================================================== */

// Keys used for the linear velocity
#define KEYCODE_w 0x77
#define KEYCODE_s 0x73
#define KEYCODE_a 0x61
#define KEYCODE_d 0x64

// Keys used for the yaw rate
#define KEYCODE_u 0x75
#define KEYCODE_o 0x6f

#define KEYCODE_R 0x52



namespace teleoperate_robot
{

using std::placeholders::_1;

class TeleopSimpleVelocityCommand : public TeleopRobot<velocity_command_msgs::msg::SimpleVelocityCommand> {
public:
    TeleopSimpleVelocityCommand();

private:
    virtual void print_instructions() override;

    virtual bool process_key(const char c) override;

    virtual void update_message() override;

    KeyboardReader keyboard_reader_;
    
    double l_scale_, a_scale_;

    double velocity_forward_ = 0;
    double velocity_lateral_ = 0;
    double yaw_rate_ = 0;
};

} // namespace_teleoperate_robot

