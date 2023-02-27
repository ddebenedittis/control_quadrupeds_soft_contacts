#include "teleoperate_robot/teleop_robot_base_node.hpp"

#include "teleoperate_robot/quat_math.h"



/* ============================== TeleopRobotx ============================== */

TeleopRobot::TeleopRobot()
: Node("teleop_robot"),
  l_twist_{0.0, 0.0, 0.0},
  a_twist_{0.0, 0.0, 0.0},
  l_scale_(1.0),
  a_scale_(1.0)
{
    this->declare_parameter("scale_angular", rclcpp::ParameterValue(1.0));
    this->declare_parameter("scale_linear", rclcpp::ParameterValue(1.0));
    this->get_parameter("scale_angular", a_scale_);
    this->get_parameter("scale_linear", l_scale_);

    des_gen_pose_pub_ = this->create_publisher<generalized_pose_msgs::msg::GeneralizedPose>(
        "robot/desired_generalized_pose", 1);

    // base_pose_sub_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
    //     "gazebo/link_states", 1, std::bind(&TeleopRobot::pose_callback, this, _1));

    timer_ = rclcpp::create_timer(this, this->get_clock(), timer_period_, std::bind(&TeleopRobot::read_key, this));

    TeleopRobot::print_instructions();
}


/* ================================ Key_read ================================ */

void TeleopRobot::read_key()
{
    char c;
    bool dirty = false;

    try {
        keyboard_reader_.read_one(&c);
    } catch (const std::runtime_error &) {
        perror("read():");
    }

    l_twist_ = a_twist_ = {0.0, 0.0, 0.0};
    RCLCPP_DEBUG(this->get_logger(), "value: 0x%02X\n", c);

    dirty = process_key(c);

    // Update the desired base position
    for(int i=0; i<static_cast<int>(r_b_des.size()); i++) {
        r_b_des[i] = r_b_des[i] + l_twist_[i] * l_scale_ * 0.01;
    }
    
    // Integrate the orientation with the angular velocity in body frame received by the user input.
    q_des = quat_int(q_des, a_twist_, a_scale_ * 0.01);


    // Initialize the message and set its parts.
    generalized_pose_msgs::msg::GeneralizedPose gen_pose;
    
    gen_pose.base_acc.x = r_b_ddot_des[0];
    gen_pose.base_acc.y = r_b_ddot_des[1];
    gen_pose.base_acc.z = r_b_ddot_des[2];

    gen_pose.base_vel.x = r_b_dot_des[0];
    gen_pose.base_vel.y = r_b_dot_des[1];
    gen_pose.base_vel.z = r_b_dot_des[2];

    gen_pose.base_pos.x = r_b_des[0];
    gen_pose.base_pos.y = r_b_des[1];
    gen_pose.base_pos.z = r_b_des[2];

    gen_pose.base_angvel.x = omega_des[0];
    gen_pose.base_angvel.y = omega_des[1];
    gen_pose.base_angvel.z = omega_des[2];

    gen_pose.base_quat.x = q_des[0];
    gen_pose.base_quat.y = q_des[1];
    gen_pose.base_quat.z = q_des[2];
    gen_pose.base_quat.w = q_des[3];

    gen_pose.contact_feet = {"LF", "RF", "LH", "RH"};


    // This if statement is used to publishes the message only when the desired generalized pose is changed (when there is a user input).
    if (dirty == true) {
        des_gen_pose_pub_->publish(gen_pose);
        // base_pose_sub_.reset();
    }
}


/* =========================== Print_instructions =========================== */

void TeleopRobot::print_instructions()
{
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
}


/* =============================== Process_key ============================== */

bool TeleopRobot::process_key(const char c)
{
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
        l_twist_ = {0.0, 1.0, 0.0};
        return true;
    case KEYCODE_a:
        RCLCPP_DEBUG(this->get_logger(), "TILT TO THE LEFT\n");
        l_twist_ = {0.0, -1.0, 0.0};
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


/* ============================ gen_pose_callback =========================== */

void TeleopRobot::pose_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
{
    const int base_id = 1;

    r_b_des = {
        msg->pose[base_id].position.x,
        msg->pose[base_id].position.y,
        msg->pose[base_id].position.z
    };

    q_des[0] = msg->pose[base_id].orientation.x;
    q_des[1] = msg->pose[base_id].orientation.y;
    q_des[2] = msg->pose[base_id].orientation.z;
    q_des[3] = msg->pose[base_id].orientation.w;
}



/* ========================================================================== */
/*                          MAIN FUNCTION DEFINITION                          */
/* ========================================================================== */

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // TeleopRobot teleop_robot;
    auto teleop_robot = std::make_shared<TeleopRobot>();

    rclcpp::spin(teleop_robot);

    teleop_robot->shutdown();

    rclcpp::shutdown();
    
    return 0;
}