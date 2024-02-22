#pragma once

#include "lip_walking_trot_planner/lip_planner.hpp"

#include "lip_walking_trot_planner/fading_filter.tpp"

#include "Eigen/Core"

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "generalized_pose_msgs/generalized_pose_struct.hpp"

#include "gazebo_msgs/msg/link_states.hpp"
#include "generalized_pose_msgs/msg/generalized_poses_with_time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rviz_legged_msgs/msg/paths.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "velocity_command_msgs/msg/simple_velocity_command.hpp"

#include <string>
#include <vector>



namespace lip_walking_trot_planner {

using namespace Eigen;


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LIPController : public controller_interface::ControllerInterface {
    public:
        LIPController() = default;

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
        /// @brief Publish the desired generalized pose message.
        void publish_gen_pose();

        void publish_feet_trajectories();

        void correct_with_terrain_plane();

        void correct_with_terrain_penetration();

        /// @brief Planner class instance.
        MotionPlanner planner_;

        FadingFilter<Vector3d> filter_;

        generalized_pose::GeneralizedPoseStruct gen_pose_;
        std::vector<generalized_pose::GeneralizedPoseStruct> gen_poses_;

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr feet_positions_subscription_ = nullptr;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr feet_velocities_subscription_ = nullptr;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_ = nullptr;

        // rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr link_states_subscription_ = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base_pose_subscription_ = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr base_twist_subscription_ = nullptr;

        rclcpp::Subscription<velocity_command_msgs::msg::SimpleVelocityCommand>::SharedPtr simple_velocity_command_subscription_ = nullptr;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr terrain_penetrations_subscription_ = nullptr;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr terrain_plane_subscription_ = nullptr;

        std::shared_ptr<rclcpp::Publisher<generalized_pose_msgs::msg::GeneralizedPose>> gen_pose_publisher_ = nullptr;
        std::shared_ptr<rclcpp::Publisher<generalized_pose_msgs::msg::GeneralizedPosesWithTime>> gen_poses_publisher_ = nullptr;
        std::shared_ptr<rclcpp::Publisher<rviz_legged_msgs::msg::Paths>> feet_trajectories_publisher_ = nullptr;

        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> base_trajectory_publisher_ = nullptr;
        std::shared_ptr<rclcpp::Publisher<rviz_legged_msgs::msg::Paths>> feet_trajectories_2_publisher_ = nullptr;

        /* ============== Variables Saved By The Subscriptions ============== */

        std::vector<Vector3d> feet_positions = {};
        std::vector<Vector3d> feet_velocities = {};

        VectorXd q_ = (VectorXd(7) << 0, 0, 0, 0, 0, 0, 1).finished();
        VectorXd v_ = VectorXd::Zero(6);
        Vector3d a_ = VectorXd::Zero(3);

        double velocity_forward_ = 0;
        double velocity_lateral_ = 0;
        double yaw_rate_ = 0;

        Vector3d plane_coeffs_ = Vector3d::Zero();

        Vector4d terrain_penetrations_ = Vector4d::Zero();

        /* =========================== Parameters =========================== */

        double zero_time_ = 0;
        double init_time_ = 0;

        double default_step_height_ = 0.1;

        bool correct_with_terrain_penetrations_ = false;
        double gain_correction_with_terrain_penetrations_ = 1;

        bool interpolate_swing_feet_from_current_position_ = false;

        /* ========================= Internal State ========================= */

        Vector3d init_pos_ = {0, 0, 0};
};

} // namespace static_walk_planner