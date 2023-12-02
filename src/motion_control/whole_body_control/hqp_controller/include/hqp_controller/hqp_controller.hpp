#pragma once

#include "hqp_controller/hqp_publisher.hpp"
#include "whole_body_controller/whole_body_controller.hpp"

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "gazebo_msgs/msg/link_states.hpp"
#include "generalized_pose_msgs/msg/generalized_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <vector>



namespace hqp_controller {

/* ========================================================================== */
/*                                HQPCONTROLLER                               */
/* ========================================================================== */

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HQPController : public controller_interface::ControllerInterface {
public:
    HQPController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& /*period*/
    ) override;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override;
    // CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    // CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    // CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

private:
    wbc::WholeBodyController wbc;

    std::vector<std::string> joint_names_;

    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    wbc::GeneralizedPose des_gen_pose_;

    Eigen::VectorXd tau_;

    double PD_proportional_ = 1;
    double PD_derivative_ = 1;

    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr joint_state_subscription_ = nullptr;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr estimated_pose_subscription_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr estimated_twist_subscription_ = nullptr;

    rclcpp::Subscription<generalized_pose_msgs::msg::GeneralizedPose>::SharedPtr desired_generalized_pose_subscription_ = nullptr;

    /// @brief If true, the controller will publish the computed optimal joint torques, contact forces, and feet deformations.
    bool logging_ = false;

    /// @brief If true, the desired base height in world frame is shifted to take into account the fact that the terrain is soft and the robot will penetrate it. It also takes into account the number of feet in contact with the terrain.
    bool shift_base_height_ = false;

    std::shared_ptr<HQPPublisher> logger_ = nullptr;

    /// @brief Initialization time to give the state estimator some time to get better estimates. During this time, a PD controller is used to keep the robot in q0 and the planner is paused.
    double init_time_ = 1;
    std::vector<double> init_phases_ = {1};

    Eigen::VectorXd q0_;
    Eigen::VectorXd q1_;
    Eigen::VectorXd q2_;
};

} // namespace hqp_controller