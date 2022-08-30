#include "teleoperate_robot/quat_math.h"

#include "rclcpp/rclcpp.hpp"

#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "generalized_pose_msgs/msg/desired_generalized_pose.hpp"

#include <array>
#include <memory.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>



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
/*                            KEYBOARDREADER CLASS                            */
/* ========================================================================== */

class KeyboardReader {
    public:
        KeyboardReader()
        : kfd(0)
        {
            // Get the console in raw mode
            tcgetattr(kfd, &cooked);
            struct termios raw;
            memcpy(&raw, &cooked, sizeof(struct termios));
            raw.c_lflag &=~ (ICANON | ECHO);
            // Setting a new line, then end of file
            raw.c_cc[VEOL] = 1;
            raw.c_cc[VEOF] = 2;
            tcsetattr(kfd, TCSANOW, &raw);
        }
        
        void readOne(char * c)
        {
            int rc = read(kfd, c, 1);
            if (rc < 0) {
                throw std::runtime_error("read failed");
            }
        }
        
        void shutdown()
        {
            tcsetattr(kfd, TCSANOW, &cooked);
        }

    private:
        int kfd;
        struct termios cooked;
};



/* ========================================================================== */
/*                              TELEOPROBOT CLASS                             */
/* ========================================================================== */

using std::placeholders::_1;

class TeleopRobot : public rclcpp::Node
{
public:
    TeleopRobot();

    /// TeleopAnymal member function that handles the keyboard input and publish the DesGenPose to the appropriate topic.
    int keyLoop();

private:
    void gen_pose_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg);
    
    std::array<double, 3> l_twist_, a_twist_;
    double l_scale_, a_scale_;

    std::array<double, 3> r_b_des;  ///< @brief Desired base position
    std::array<double, 4> q_des;    ///< @brief Desired base orientation

    rclcpp::Publisher<generalized_pose_msgs::msg::DesiredGeneralizedPose>::SharedPtr des_gen_pose_pub_;

    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr base_pose_sub_;
};


/* ============================ Class Constructor =========================== */

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

    des_gen_pose_pub_ = this->create_publisher<generalized_pose_msgs::msg::DesiredGeneralizedPose>(
        "robot/desired_generalized_pose", 1);

    base_pose_sub_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
        "gazebo/link_states", 1, std::bind(&TeleopRobot::gen_pose_callback, this, _1));
}


/* ============================ gen_pose_callback =========================== */

void TeleopRobot::gen_pose_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
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
/*                          QUIT FUNCTION DEFINITION                          */
/* ========================================================================== */

// Function that should reset some terminal parameters that have been modified when this node runs.

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}



/* ========================================================================== */
/*                          MAIN FUNCTION DEFINITION                          */
/* ========================================================================== */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    TeleopRobot teleop_robot;

    signal(SIGINT,quit);

    int rc = teleop_robot.keyLoop();
    quit(0);
    
    return rc;
}



/* ========================================================================== */
/*                                   KEYLOOP                                  */
/* ========================================================================== */

int TeleopRobot::keyLoop()
{
    char c;
    bool dirty=false;

    // Some user instructions
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use the following keys (CASE SENSITIVE!) to move the robot base:");
    puts("W: tilt forward,          S: tilt backward");
    puts("d: tilt to the right,     a: tilt to the left");
    puts("q: tilt up,               e: tilt down");
    puts("---------------------------");
    puts("Use the following keys (CASE SENSITIVE!) to rotate the robot base:");
    puts("i: positive pitch,        k: negative pitch");
    puts("l: positive roll,         j: negative roll");
    puts("u: positive yaw,          o: negative yaw");


    // Initialize the desired generalized pose components
    std::array<double, 3> r_b_ddot_des, r_b_dot_des;
    std::array<double, 3> omega_des;

    // Assign the initial values of the desired generalized pose
    r_b_ddot_des = r_b_dot_des = {0.0, 0.0, 0.0};
    omega_des = {0.0, 0.0, 0.0};

    r_b_des = {0.0, 0.0, 0.0};
    q_des = {10.0, 0.0, 0.0, 0.0};

    // Block until the base initial pose has been received
    RCLCPP_INFO(this->get_logger(), "Waiting for Gazebo to start.\n");

    for(;;) {
        if(q_des[0] > 5.0) {
            sleep(0.001);
        } else {
            base_pose_sub_.reset();
            break;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Gazebo started.\n");

    // Set the desired base height to 0.5
    r_b_des[2] = 0.5;


    for(;;) {
        // get the next event from the keyboard  
        try {
            input.readOne(&c);
        } catch (const std::runtime_error &) {
            perror("read():");
            return -1;
        }

        l_twist_ = a_twist_ = {0.0, 0.0, 0.0};
        RCLCPP_DEBUG(this->get_logger(), "value: 0x%02X\n", c);
    
        // switch-case that associates the keys to the corresponding action
        switch(c) {
            // Linear movements of the base
            case KEYCODE_w:
                RCLCPP_DEBUG(this->get_logger(), "TILT FORWARD\n");
                l_twist_ = {1.0, 0.0, 0.0};
                dirty = true;
                break;
            case KEYCODE_s:
                RCLCPP_DEBUG(this->get_logger(), "TILT BACKWARD\n");
                l_twist_ = {-1.0, 0.0, 0.0};
                dirty = true;
                break;
            case KEYCODE_d:
                RCLCPP_DEBUG(this->get_logger(), "TILT TO THE RIGHT\n");
                l_twist_ = {0.0, 1.0, 0.0};
                dirty = true;
                break;
            case KEYCODE_a:
                RCLCPP_DEBUG(this->get_logger(), "TILT TO THE LEFT\n");
                l_twist_ = {0.0, -1.0, 0.0};
                dirty = true;
                break;
            case KEYCODE_q:
                RCLCPP_DEBUG(this->get_logger(), "TILT UP\n");
                l_twist_ = {0.0, 0.0, 1.0};
                dirty = true;
                break;
            case KEYCODE_e:
                RCLCPP_DEBUG(this->get_logger(), "TILT DOWN\n");
                l_twist_ = {0.0, 0.0, -1.0};
                dirty = true;
                break;

            // Angular movements of the base
            case KEYCODE_l:
                RCLCPP_DEBUG(this->get_logger(), "POSITIVE ROLL\n");
                a_twist_ = {1.0, 0.0, 0.0};
                dirty = true;
                break;
            case KEYCODE_j:
                RCLCPP_DEBUG(this->get_logger(), "NEGATIVE ROLL\n");
                a_twist_ = {-1.0, 0.0, 0.0};
                dirty = true;
                break;
            case KEYCODE_i:
                RCLCPP_DEBUG(this->get_logger(), "NEGATIVE PITCH\n");
                a_twist_ = {0.0, -1.0, 0.0};
                dirty = true;
                break;
            case KEYCODE_k:
                RCLCPP_DEBUG(this->get_logger(), "POSITIVE PITCH\n");
                a_twist_ = {0.0, 1.0, 0.0};
                dirty = true;
                break;
            case KEYCODE_u:
                RCLCPP_DEBUG(this->get_logger(), "POSITIVE YAW\n");
                a_twist_ = {0.0, 0.0, 1.0};
                dirty = true;
                break;
            case KEYCODE_o:
                RCLCPP_DEBUG(this->get_logger(), "NEGATIVE YAW\n");
                a_twist_ = {0.0, 0.0, -1.0};
                dirty = true;
                break;
        }


        // Stupid C++ does not even know how to sum two two arrays. 
        // This for cycle is necessary to sum two std::arrays. If this must be done often, a function that automatically does this could be created.
        for(int i=0; i<static_cast<int>(r_b_des.size()); i++) {
            r_b_des[i] = r_b_des[i] + l_twist_[i] * l_scale_ * 0.01;
        }
        
        // Integrate the orientation with the angular velocity in body frame received by the user input.
        q_des = quat_int(q_des, a_twist_, a_scale_ * 0.01);


        // Initialize the message and set its parts.
        generalized_pose_msgs::msg::DesiredGeneralizedPose gen_pose;
        
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


        // This if statement is used to publishes the message only when the desired generalized pose is changed (when there is a user input).
        if(dirty ==true) {
            des_gen_pose_pub_->publish(gen_pose);    
            dirty=false;
        }
    }

    return 0;
}