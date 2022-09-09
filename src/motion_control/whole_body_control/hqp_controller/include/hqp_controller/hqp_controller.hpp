#pragma once

#include "whole_body_controller/whole_body_controller.hpp"

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "gazebo_msgs/msg/link_states.hpp"
#include "generalized_pose_msgs/msg/desired_generalized_pose.hpp"

#include <string>
#include <vector>

namespace hqp_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HQPController : public controller_interface::ControllerInterface {
    public:
        HQPController();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period
        ) override;

        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        // CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        // CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
        // CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;



    protected:
        wbc::WholeBodyController wbc;

        std::vector<std::string> joint_names_;

        Eigen::VectorXd q_;
        Eigen::VectorXd v_;
        wbc::GeneralizedPose des_gen_pose_;

        Eigen::VectorXd tau_;

        rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr joint_state_subscription_ = nullptr;

        rclcpp::Subscription<generalized_pose_msgs::msg::DesiredGeneralizedPose>::SharedPtr desired_generalized_pose_subscription_ = nullptr;
};

} // namespace hqp_controller