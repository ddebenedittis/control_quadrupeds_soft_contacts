#include "teleoperate_robot/keyboard_reader.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "generalized_pose_msgs/msg/generalized_pose.hpp"

#include <array>



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



/* ========================================================================== */
/*                              TELEOPROBOT CLASS                             */
/* ========================================================================== */

using std::placeholders::_1;

class TeleopRobot : public rclcpp::Node {
public:
    TeleopRobot();

    /// TeleopRobot member function that handles the keyboard input and publish the DesGenPose to the appropriate topic.
    void read_key();

    void shutdown() {keyboard_reader_.shutdown();};

private:
    void print_instructions();

    bool process_key(const char c);

    void pose_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg);

    KeyboardReader keyboard_reader_;
    
    std::array<double, 3> l_twist_, a_twist_;
    double l_scale_, a_scale_;

    std::array<double, 3> r_b_des = {0., 0., 0.55};      ///< @brief Desired base position
    std::array<double, 4> q_des = {0., 0., 0., 1.};     ///< @brief Desired base orientation

    std::array<double, 3> r_b_dot_des = {0., 0., 0.};
    std::array<double, 3> r_b_ddot_des = {0., 0., 0.};
    std::array<double, 3> omega_des = {0., 0., 0.};

    rclcpp::Publisher<generalized_pose_msgs::msg::GeneralizedPose>::SharedPtr des_gen_pose_pub_;

    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr base_pose_sub_;

    const rclcpp::Duration timer_period_ = rclcpp::Duration::from_seconds(1./25.);
    rclcpp::TimerBase::SharedPtr timer_;
};