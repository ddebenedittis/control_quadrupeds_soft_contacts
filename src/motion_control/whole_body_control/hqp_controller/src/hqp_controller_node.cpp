#include "whole_body_controller/whole_body_controller.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gazebo_msgs/msg/link_states.hpp"
#include "generalized_pose_msgs/msg/desired_generalized_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

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
            std::vector<std::string> all_feet_names,
            int base_id=1)
        : Node("minimal_subscriber"),
          base_id(base_id)
        {
            q = Eigen::VectorXd::Zero(nv+1);
            v = Eigen::VectorXd::Zero(nv);

            nj = nv - 6;

            gen_pose.base_acc = {0, 0, 0};
            gen_pose.base_vel = {0, 0, 0};
            gen_pose.base_pos = {0, 0, 0.55};
            gen_pose.base_angvel = {0, 0, 0};
            gen_pose.base_quat = {0, 0, 0, 1};
            gen_pose.feet_acc = {};
            gen_pose.feet_vel = {};
            gen_pose.feet_pos = {};
            gen_pose.contact_feet_names = all_feet_names;

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
            std::vector<std::string> joint_names = {"LF_HAA", "LF_HFE", "LF_KFE",
                                                    "LH_HAA", "LH_HFE", "LH_KFE",
                                                    "RF_HAA", "RF_HFE", "RF_KFE",
                                                    "RH_HAA", "RH_HFE", "RH_KFE"};
            
            std::vector<std::string>::iterator it;

            for(unsigned int i = 0; i < nj; i++) {
                it = find(joint_names.begin(), joint_names.end(), msg->name[i]);

                q(7 + static_cast<int>(it - joint_names.begin())) = msg->position[i];
                v(6 + static_cast<int>(it - joint_names.begin())) = msg->velocity[i];
            }
            
            // q.tail(nj) = Eigen::VectorXd::Map(&msg->position[0], msg->position.size());

            // v.tail(nj) = Eigen::VectorXd::Map(&msg->velocity[0], msg->velocity.size());
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

            gen_pose.contact_feet_names.assign(&msg->contact_feet[0], &msg->contact_feet[msg->contact_feet.size()]);
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
        rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr link_state_subscription_;
        rclcpp::Subscription<generalized_pose_msgs::msg::DesiredGeneralizedPose>::SharedPtr desired_generalized_pose_subscription_;

        /// @brief Number of robot joints
        int nj;

        /// @brief Id of the base joint
        int base_id;

        /// @brief Generalized coordinate vector
        /// @details q = [pos_base; quat_base; q_joints] = [base position in inertial frame; base_orientation (unit quaternion); joints angles]
        Eigen::VectorXd q;

        /// @brief Generalized velocity vector
        /// @details v = [v_base; omega_base; q_dot_joints] = [base linear velocity in inertial frame; base angular velocity w.r.t. inertial frame expressed in base frame; joints velocities]
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
                "/effort_controller/commands", 1);
        }

        void publish_torques(Eigen::VectorXd& torques)
        {
            // Convert the torques to a ROS message and publish it

            auto  message = std_msgs::msg::Float64MultiArray();
            message.data = std::vector<double>(torques.data(), torques.data() + torques.size());
            torques_publisher_->publish(message);
        }

    private:
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torques_publisher_;
};



/* ========================================================================== */
/*                                    MAIN                                    */
/* ========================================================================== */

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    

    /* ==================== Access The Parameters Passed ==================== */

    auto nh = rclcpp::Node::make_shared("_");
    nh->declare_parameter<std::string>("robot_name");
    rclcpp::Parameter robot_name_param = nh->get_parameter("robot_name");
    std::string robot_name = robot_name_param.as_string();


    // Controller time step
    float dt = 1./400.;

    rclcpp::Rate loop_rate(1/dt);


    /* ================== Instantiate The Controller Class ================== */

    wbc::WholeBodyController wbc(robot_name, dt);

    wbc.set_kp_b_pos(100 * Eigen::Vector3d(1,1,1));
    wbc.set_kd_b_pos( 10 * Eigen::Vector3d(1,1,1));

    wbc.set_kp_b_ang(150 * Eigen::Vector3d(1,1,1));
    wbc.set_kd_b_ang( 35 * Eigen::Vector3d(1,1,1));

    wbc.set_kp_s_pos(150 * Eigen::Vector3d(1,1,1));
    wbc.set_kd_s_pos( 30 * Eigen::Vector3d(1,1,1));

    wbc.set_Kp_terr(1000 * Eigen::Vector3d(1,1,1));
    wbc.set_Kd_terr(1000 * Eigen::Vector3d(1,1,1));


    /* ============= Instantite The Subscriber And The Publisher ============ */

    int nv = wbc.get_nv();

    // Define the topic names
    const char* joint_state_topic_name = "/joint_states";
    const char* link_states_topic_name = "/gazebo/link_states";
    const char* desired_generalized_state_topic_name = "/robot/desired_generalized_pose";
    
    auto subscr = std::make_shared<MinimalSubscriber>(
        nv,
        joint_state_topic_name,
        link_states_topic_name,
        desired_generalized_state_topic_name,
        wbc.get_all_feet_names()
    );

    std::thread thread([](auto subscr){rclcpp::spin(subscr);}, subscr);     // spin the node in another thread in order to not block

    // Instantiate the publisher
    auto publsh = std::make_shared<MinimalPublisher>();


    /* ================== Block Until The Simulation Starts ================= */

    Eigen::VectorXd q;
    Eigen::VectorXd v;
    wbc::GeneralizedPose des_gen_pose;

    Eigen::VectorXd tau;

    // Exit this cycle only when the orientation norm is greater than 0 (it means that a link_states message has been received)
    while (rclcpp::ok()) {
        subscr->get_all(q, v, des_gen_pose);

        if (q.segment(3, 4).squaredNorm() > 0) {
            break;
        }

        loop_rate.sleep();
    }


    /* ============================== Main Loop ============================= */

    while (rclcpp::ok()) {
        // Get the relevant values received by the subscriber
        subscr->get_all(q, v, des_gen_pose);

        // Compute the optimal torques
        wbc.step(q, v, des_gen_pose);

        // Get the optimal torques
        tau = wbc.get_tau_opt();

        // Publish the optimal torques
        publsh->publish_torques(tau);

        // Sleep for the remainder of the timestep
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    thread.join();
    return 0;
}