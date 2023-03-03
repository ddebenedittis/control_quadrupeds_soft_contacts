#pragma once

#include "teleoperate_robot/teleop_robot.tpp"

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "generalized_pose_msgs/msg/generalized_pose.hpp"

#include <array>
#include <string>



/* ========================================================================== */
/*                            CONSTANTS DEFINITION                            */
/* ========================================================================== */

// Define all the keys used for the teleoperation of anymal
// These values can be obtained in terminal with the command showkey -a
// Attention! Be careful not to have block maiusc activated when teleoperating the robot.

// Keys used for the linear part of the pose
#define KEYCODE_w 0x77
#define KEYCODE_s 0x73
#define KEYCODE_a 0x61
#define KEYCODE_d 0x64
#define KEYCODE_q 0x71
#define KEYCODE_e 0x65

// Keys used for the angular part of the pose
#define KEYCODE_i 0x69
#define KEYCODE_k 0x6b
#define KEYCODE_j 0x6a
#define KEYCODE_l 0x6c
#define KEYCODE_u 0x75
#define KEYCODE_o 0x6f



namespace teleoperate_robot
{

using std::placeholders::_1;

class TeleopRobotBase : public TeleopRobot<generalized_pose_msgs::msg::GeneralizedPose> {
public:
    TeleopRobotBase();

private:
    virtual void print_instructions() override;

    virtual bool process_key(const char c) override;

    virtual void update_message() override;

    bool initialize_message() override;

    void pose_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

    KeyboardReader keyboard_reader_;
    
    std::array<double, 3> l_twist_, a_twist_;
    double l_scale_, a_scale_;

    std::array<double, 3> base_pos_des_ = {0., 0., 0.55};       ///< @brief Desired base position
    std::array<double, 4> base_quat_des_ = {0., 0., 0., 0.};    ///< @brief Desired base orientation {x, y, z, w}

    std::string robot_name;

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr base_pose_sub_;

    rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
};

} // teleoperate_robot