#include "hqp_controller/hqp_publisher.hpp"



namespace hqp_controller {

/* ========================================================================== */
/*                                HQPPUBLISHER                                */
/* ========================================================================== */

/* =============================== Constructor ============================== */

HQPPublisher::HQPPublisher(const std::vector<std::string> feet_names)
: Node("HQP_publisher")
{
    feet_names_ = feet_names;

    joints_accelerations_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/optimal_joints_accelerations", 1);

    torques_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/optimal_torques", 1);

    forces_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/optimal_forces", 1);

    deformations_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/optimal_deformations", 1);


    feet_positions_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/feet_positions", 1);

    feet_velocities_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/logging/feet_velocities", 1);


    wrenches_stamped_publisher_ = this->create_publisher<rviz_legged_msgs::msg::WrenchesStamped>(
        "/logging/wrenches_stamped", 1);

    polygon_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
        "/logging/polygon_stamped", 1);

    friction_cones_publisher_ = this->create_publisher<rviz_legged_msgs::msg::FrictionCones>(
        "/rviz/friction_cones", 1);
}


/* ======================= Publish_float64_multi_array ====================== */

inline void HQPPublisher::publish_float64_multi_array(
    const Eigen::VectorXd& vector,
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher)
{
    auto message = std_msgs::msg::Float64MultiArray();
    message.data = std::vector<double>(vector.data(), vector.data() + vector.size());
    publisher->publish(message);
}


/* ======================== Publish_wrenches_stamped ======================== */

inline void HQPPublisher::publish_wrenches_stamped(const Eigen::VectorXd& forces)
{
    auto wrenches_stamped_message = rviz_legged_msgs::msg::WrenchesStamped();
    wrenches_stamped_message.header.frame_id = "ground_plane_link";
    wrenches_stamped_message.wrenches_stamped.resize(4);
    for (int i = 0; i < 4; i++) {
        wrenches_stamped_message.wrenches_stamped[i].header.frame_id = feet_names_[i];
        wrenches_stamped_message.wrenches_stamped[i].wrench.force.x = forces[0 + 3*i];
        wrenches_stamped_message.wrenches_stamped[i].wrench.force.y = forces[1 + 3*i];
        wrenches_stamped_message.wrenches_stamped[i].wrench.force.z = forces[2 + 3*i];
    }
    wrenches_stamped_publisher_->publish(wrenches_stamped_message);
}


/* =================== Publish_polygon_and_friction_cones =================== */

inline void HQPPublisher::publish_polygon_and_friction_cones(
    const Eigen::VectorXd& feet_positions, const std::vector<std::string>& contact_feet_names,
    const std::vector<std::string>& all_generic_feet_names, const std::vector<std::string>& all_specific_feet_names,
    const double friction_coefficient)
{
    // Publish the support polygon message.

    auto polygon_stamped_message = geometry_msgs::msg::PolygonStamped();
    polygon_stamped_message.polygon.points.resize(contact_feet_names.size());

    std::vector<int> contact_feet_indices(contact_feet_names.size());
    for (int i = 0; i < static_cast<int>(contact_feet_names.size()); i++) {
        auto index = std::find(all_generic_feet_names.begin(), all_generic_feet_names.end(), contact_feet_names[i]);
        if (index != all_generic_feet_names.end()) {
            contact_feet_indices[i] = index - all_generic_feet_names.begin();
        }
    }

    for (auto i = 0; i < static_cast<int>(contact_feet_names.size()); i++) {
        polygon_stamped_message.header.frame_id = "ground_plane_link";
        polygon_stamped_message.polygon.points[i].x = feet_positions[0 + 3*contact_feet_indices[i]];
        polygon_stamped_message.polygon.points[i].y = feet_positions[1 + 3*contact_feet_indices[i]];
        polygon_stamped_message.polygon.points[i].z = feet_positions[2 + 3*contact_feet_indices[i]];
    }

    // LF -> RF -> LH -> RH would not produce a quadrilateral. Change the order of points.
    if (contact_feet_names.size() == 4) {
        auto temp = polygon_stamped_message.polygon.points[2];
        polygon_stamped_message.polygon.points[2] = polygon_stamped_message.polygon.points[3];
        polygon_stamped_message.polygon.points[3] = temp;
    }
    polygon_stamped_publisher_->publish(polygon_stamped_message);


    // Publish the friction cones message.

    auto friction_cones_message = rviz_legged_msgs::msg::FrictionCones();
    friction_cones_message.header.frame_id = "ground_plane_link";
    friction_cones_message.friction_cones.resize(contact_feet_names.size());
    for (auto i = 0; i < static_cast<int>(contact_feet_names.size()); i++) {
        friction_cones_message.friction_cones[i].header.frame_id = all_specific_feet_names[contact_feet_indices[i]];
        friction_cones_message.friction_cones[i].friction_coefficient = friction_coefficient;
        friction_cones_message.friction_cones[i].normal_direction.x = 0.;
        friction_cones_message.friction_cones[i].normal_direction.y = 0.;
        friction_cones_message.friction_cones[i].normal_direction.z = 1.;
    }
    friction_cones_publisher_->publish(friction_cones_message);
}


/* =============================== Publish_all ============================== */

void HQPPublisher::publish_all(
    const Eigen::VectorXd& joints_accelerations, const Eigen::VectorXd& torques,
    const Eigen::VectorXd& forces, const Eigen::VectorXd& deformations,
    const Eigen::VectorXd& feet_positions, const Eigen::VectorXd& feet_velocities,
    const std::vector<std::string>& contact_feet_names, const std::vector<std::string>& all_generic_feet_names,
    const std::vector<std::string>& all_specific_feet_names, const double friction_coefficient)
{
    publish_float64_multi_array(joints_accelerations, joints_accelerations_publisher_);
    publish_float64_multi_array(torques, torques_publisher_);
    publish_float64_multi_array(forces, forces_publisher_);
    publish_float64_multi_array(deformations, deformations_publisher_);

    publish_float64_multi_array(feet_positions, feet_positions_publisher_);
    publish_float64_multi_array(feet_velocities, feet_velocities_publisher_);

    publish_wrenches_stamped(forces);

    publish_polygon_and_friction_cones(
        feet_positions, contact_feet_names,
        all_generic_feet_names, all_specific_feet_names,
        friction_coefficient);
}

} // hqp_controller