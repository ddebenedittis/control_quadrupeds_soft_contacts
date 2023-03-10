/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rviz_legged_plugins/displays/friction_cones_display.hpp"

#include <limits>
#include <memory>

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/parse_color.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>



namespace rviz_legged_plugins
{
namespace displays
{

FrictionConesDisplay::FrictionConesDisplay(rviz_common::DisplayContext * display_context)
: FrictionConesDisplay()
{
    context_ = display_context;
    scene_manager_ = context_->getSceneManager();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    updateBufferLength();
}

FrictionConesDisplay::FrictionConesDisplay()
{
    height_property_ = new rviz_common::properties::FloatProperty(
        "Height", 0.2f,
        "Height of the cone.",
        this, SLOT(updateColorAndAlpha()));

    color_property_ = new rviz_common::properties::ColorProperty(
        "Color", Qt::white,
        "Color to draw the range.",
        this, SLOT(updateColorAndAlpha()));

    alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 0.5f,
        "Amount of transparency to apply to the range.",
        this, SLOT(updateColorAndAlpha()));

    buffer_length_property_ = new rviz_common::properties::IntProperty(
        "Buffer Length", 1,
        "Number of prior measurements to display.",
        this, SLOT(updateBufferLength()));
    buffer_length_property_->setMin(1);
}

void FrictionConesDisplay::onInitialize()
{
    MFDClass::onInitialize();
    updateBufferLength();
    updateColorAndAlpha();
}

FrictionConesDisplay::~FrictionConesDisplay() = default;

void FrictionConesDisplay::reset()
{
    MFDClass::reset();
    updateBufferLength();
}

void FrictionConesDisplay::updateColorAndAlpha()
{
    auto color = color_property_->getOgreColor();
    float alpha = alpha_property_->getFloat();
    for (const auto & cone : cones_) {
        cone->setColor(color.r, color.g, color.b, alpha);
    }
    context_->queueRender();
}

void FrictionConesDisplay::updateBufferLength()
{
    int buffer_length = number_cones_ * buffer_length_property_->getInt();
    auto color = color_property_->getOgreColor();
    cones_.resize(buffer_length);

    for (auto & cone : cones_) {
        cone.reset(
        new rviz_rendering::Shape(
            rviz_rendering::Shape::Cone, context_->getSceneManager(), scene_node_));

        cone->setScale(Ogre::Vector3(0, 0, 0));
        cone->setColor(color.r, color.g, color.b, 0);
    }
}

void FrictionConesDisplay::processMessage(const rviz_legged_msgs::msg::FrictionCones::ConstSharedPtr msg)
{
    number_cones_ = msg->friction_cones.size();
    updateBufferLength();

    for (int i = 0; i < number_cones_; i++) {
        auto friction_cone_msg = msg->friction_cones[i];

        auto cone = cones_[i];

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        float displayed_range = height_property_->getFloat();
        auto pose = getPose();

        if (!context_->getFrameManager()->transform(
            friction_cone_msg.header.frame_id, friction_cone_msg.header.stamp, pose, position, orientation))
        {
            setMissingTransformToFixedFrame(friction_cone_msg.header.frame_id);
            return;
        }
        setTransformOk();

        position.x += friction_cone_msg.normal_direction.x * displayed_range/2;
        position.y += friction_cone_msg.normal_direction.y * displayed_range/2;
        position.z += friction_cone_msg.normal_direction.z * displayed_range/2;
        cone->setPosition(position);

        Eigen::Vector3d a;
        a << 0, -1, 0;
        Eigen::Vector3d b;
        b[0] = friction_cone_msg.normal_direction.x;
        b[1] = friction_cone_msg.normal_direction.y;
        b[2] = friction_cone_msg.normal_direction.z;
        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(a, b);
        orientation.x = quat.x();
        orientation.y = quat.y();
        orientation.z = quat.z();
        orientation.w = quat.w();
        cone->setOrientation(orientation);

        float cone_width = 2.0f * displayed_range * friction_cone_msg.friction_coefficient;
        Ogre::Vector3 scale(cone_width, displayed_range, cone_width);
        cone->setScale(scale);

        auto color = color_property_->getOgreColor();
        cone->setColor(color.r, color.g, color.b, alpha_property_->getFloat());
    }
}

geometry_msgs::msg::Pose FrictionConesDisplay::getPose(/*float displayed_range*/)
{
    // float fudge_factor = 0.008824f;  // fudge factor measured, must be inaccuracy of cone model.
    geometry_msgs::msg::Pose pose;

    // pose.position.x = displayed_range / 2 - fudge_factor * displayed_range;
    // pose.orientation.z = 0.707f;
    // pose.orientation.w = 0.707f;
    pose.orientation.w = 1.f;

    return pose;
}

}  // namespace displays
}  // namespace rviz_legged_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_legged_plugins::displays::FrictionConesDisplay, rviz_common::Display)