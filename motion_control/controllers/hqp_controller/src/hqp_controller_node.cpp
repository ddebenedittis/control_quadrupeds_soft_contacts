#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "generalized_pose_msgs/msg/desired_generalized_pose.hpp"

#include "whole_body_controller/whole_body_controller.hpp"

#include <Eigen/Core>

#include <memory>



/* ========================================================================== */
/*                           MINIMALSUBSCRIBER CLASS                          */
/* ========================================================================== */

using std::placeholders::_1;

/// @class @brief Class that contains the subscriber to the joint_states topic, the link_states topic, and the desired_generalized_pose topic. It stores the joints coordinates and velocity vector and the desired generalized pose computed by the planner, which can be returned by the get_all method.
class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber(
            int nv,
            const char* joint_state_topic_name,
            const char* link_states_topic_name,
            const char* desired_generalized_state_topic_name,
            int base_id=1)
        : Node("minimal_subscriber"),
          q(nv+1),
          v(nv),
          nj(nv-6),
          base_id(base_id)
        {
            joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_name, 10, std::bind(&MinimalSubscriber::joint_state_callback, this, _1));

            link_state_subscription_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
            link_states_topic_name, 10, std::bind(&MinimalSubscriber::link_states_callback, this, _1));

            desired_generalized_pose_subscription_ = this->create_subscription<generalized_pose_msgs::msg::DesiredGeneralizedPose>(
            desired_generalized_state_topic_name, 10, std::bind(&MinimalSubscriber::desired_generalized_pose_callback, this, _1));
        }

        ///@brief Get the joints coordinates and velocity vectors, and the generalized desired pose of the planner.
        ///
        ///@param q 
        ///@param v 
        ///@param gen_pose 
        void get_all(Eigen::VectorXd& q, Eigen::VectorXd& v, wbc::GeneralizedPose& gen_pose)
        {
            q = this->q;
            v = this->v;
            gen_pose = this->gen_pose;
        }

    private:
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            q.tail(nj) = Eigen::VectorXd::Map(&msg->position[0], msg->position.size());

            v.tail(nj) = Eigen::VectorXd::Map(&msg->velocity[0], msg->velocity.size());
        }

        void link_states_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
        {
            geometry_msgs::msg::Point pos = msg->pose[base_id].position;
            geometry_msgs::msg::Quaternion orient = msg->pose[base_id].orientation;

            geometry_msgs::msg::Vector3 lin = msg->twist[base_id].linear;
            geometry_msgs::msg::Vector3 ang = msg->twist[base_id].angular;

            q.head(7) << pos.x, pos.y, pos.z,
                         orient.x, orient.y, orient.z, orient.w;

            v.head(6) << lin.x, lin.y, lin.z,
                         ang.x, ang.y, ang.z;
        }

        void desired_generalized_pose_callback(const generalized_pose_msgs::msg::DesiredGeneralizedPose::SharedPtr msg)
        {
            gen_pose.base_acc << msg->base_acc.x, msg->base_acc.y, msg->base_acc.z;
            gen_pose.base_vel << msg->base_vel.x, msg->base_vel.y, msg->base_vel.z;
            gen_pose.base_pos << msg->base_pos.x, msg->base_pos.y, msg->base_pos.z;

            gen_pose.base_angvel << msg->base_angvel.x, msg->base_angvel.y, msg->base_angvel.z;
            gen_pose.base_quat << msg->base_quat.x, msg->base_quat.y, msg->base_quat.z, msg->base_quat.w;

            gen_pose.feet_acc = Eigen::VectorXd::Map(&msg->feet_acc[0], msg->feet_acc.size());
            gen_pose.feet_vel = Eigen::VectorXd::Map(&msg->feet_vel[0], msg->feet_vel.size());
            gen_pose.feet_pos = Eigen::VectorXd::Map(&msg->feet_pos[0], msg->feet_pos.size());
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
        rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr link_state_subscription_;
        rclcpp::Subscription<generalized_pose_msgs::msg::DesiredGeneralizedPose>::SharedPtr desired_generalized_pose_subscription_;

        /// @brief Number of robot joints
        int nj;

        /// @brief Id of the base joint
        int base_id;

        /// @brief Generalized coordinate vector
        Eigen::VectorXd q;

        /// @brief Generalized velocity vector
        Eigen::VectorXd v;

        /// @brief Generalized desired pose
        wbc::GeneralizedPose gen_pose;
};



/* ========================================================================== */
/*                           MINIMALPUBLISHER CLASS                           */
/* ========================================================================== */

/// @class @brief Class used to publish the joints torques in the appropriate topic
class MinimalPublisher : public rclcpp::Node
{
    public:
        MinimalPublisher()
        : Node("Minimal_publisher")
        {
            torques_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "robot/joints_torque_controller/command", 1);
        }

        void publish_torques(Eigen::VectorXd& torques)
        {
            auto  message = std_msgs::msg::Float64MultiArray();
            message.data = std::vector<double>(torques.data(), torques.data() + torques.size());
            torques_publisher_->publish(message);
        }

    private:
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torques_publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    const char* robot_name = "anymal_c";

    double dt = 1./400.;

    

    rclcpp::Rate loop_rate(1/dt);

    wbc::WholeBodyController wbc(robot_name, dt);

    int nv = wbc.get_nv();

    const char* joint_state_topic_name = "/robot/joint_states";
    const char* link_states_topic_name = "/gazebo/link_states";
    const char* desired_generalized_state_topic_name = "/robot/desired_generalized_pose";
    
    // 
    auto subscr = std::make_shared<MinimalSubscriber>(
        nv,
        joint_state_topic_name,
        link_states_topic_name,
        desired_generalized_state_topic_name
    );

    rclcpp::spin(subscr);

    while (rclcpp::ok()) {
        // subscr->get_all();

        loop_rate.sleep();
    }

    while (rclcpp::ok()) {
        // TODO: put the controller here

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}