#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rviz_legged_msgs/msg/friction_cones.hpp"
#include "rviz_legged_msgs/msg/wrenches_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>



namespace hqp_controller {

/* ========================================================================== */
/*                                HQPPUBLISHER                                */
/* ========================================================================== */

class HQPPublisher : public rclcpp::Node {
public:
    HQPPublisher(const std::vector<std::string>& feet_names);

    void publish_all(
        const Eigen::VectorXd& joints_accelerations, const Eigen::VectorXd& torques,
        const Eigen::VectorXd& forces, const Eigen::VectorXd& deformations,
        const Eigen::VectorXd& feet_positions, const Eigen::VectorXd& feet_velocities,
        const std::vector<std::string>& contact_feet_names, const std::vector<std::string>& generic_feet_names,
        const std::vector<std::string>& specific_feet_names, const double friction_coefficient,
        const Eigen::Vector3d& com_position);

private:
    static inline void publish_float64_multi_array(
        const Eigen::VectorXd& vector,
        const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher);

    inline void publish_wrenches_stamped(const Eigen::VectorXd& forces);

    inline void publish_polygon_and_friction_cones(
        const Eigen::VectorXd& feet_positions, const std::vector<std::string>& contact_feet_names,
        const std::vector<std::string>& all_generic_feet_names, const std::vector<std::string>& all_specific_feet_names,
        const double friction_coefficient);

    inline void publish_point(const Eigen::Vector3d& point);

    std::vector<std::string> feet_names_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joints_accelerations_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torques_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forces_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr deformations_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feet_positions_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feet_velocities_publisher_;

    rclcpp::Publisher<rviz_legged_msgs::msg::WrenchesStamped>::SharedPtr wrenches_stamped_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_stamped_publisher_;
    rclcpp::Publisher<rviz_legged_msgs::msg::FrictionCones>::SharedPtr friction_cones_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr com_publisher_;
};

} // hqp_controller