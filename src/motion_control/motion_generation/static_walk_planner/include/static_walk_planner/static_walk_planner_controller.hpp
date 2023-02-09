#pragma once

#include "static_walk_planner/static_walk_planner.hpp"

#include "Eigen/Core"

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "generalized_pose_msgs/msg/generalized_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <vector>



namespace static_walk_planner {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SWPController : public controller_interface::ControllerInterface {
    public:
        SWPController();

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

    protected:
        /// @brief Publish the desired generalized pose message.
        void publish_gen_pose();

        /// @brief Planner class instance.
        static_walk_planner::StaticWalkPlanner planner_;

        static_walk_planner::GeneralizedPose gen_pose_;

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr estimated_pose_subscription_ = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr estimated_twist_subscription_ = nullptr;
        Eigen::VectorXd q_;
        Eigen::VectorXd q_init_;

        std::shared_ptr<rclcpp::Publisher<generalized_pose_msgs::msg::GeneralizedPose>> gen_pose_publisher_ = nullptr;

        /// @brief Initialization time to give the state estimator some time to get better estimates. During this time, a PD controller is used to keep the robot in q0 and the planner is paused.
        double init_time_ = 1;

        double last_time_ = 0;
};

} // namespace static_walk_planner