/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#pragma once

#include <memory>
#include <vector>

#include "rviz_legged_msgs/msg/friction_cones.hpp"

#include "rviz_common/message_filter_display.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{
class Shape;
}  // namespace rviz_rendering

namespace rviz_common
{
class QueueSizeProperty;
namespace properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}  // namespace properties
}  // namespace rviz_common


namespace rviz_legged_plugins
{
namespace displays
{
/**
 * \class FrictionConesDisplay
 * \brief Displays a the friction cones of the feet in contact with the terrain.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC FrictionConesDisplay : public
    rviz_common::MessageFilterDisplay<rviz_legged_msgs::msg::FrictionCones>
{
    Q_OBJECT

public:
    // TODO(botteroa-si): Constructor for testing. Remove once ros nodes can be mocked and
    // initialize() can be called
    explicit FrictionConesDisplay(rviz_common::DisplayContext * display_context);

    FrictionConesDisplay();

    ~FrictionConesDisplay() override;

    void reset() override;

    void processMessage(rviz_legged_msgs::msg::FrictionCones::ConstSharedPtr msg) override;

protected:
    void onInitialize() override;

private Q_SLOTS:
    void updateBufferLength();
    void updateColorAndAlpha();

private:
    int number_cones_ = 1;

    float getDisplayedRange(rviz_legged_msgs::msg::FrictionCones::ConstSharedPtr msg);
    geometry_msgs::msg::Pose getPose(/*float displayed_range*/);

    std::vector<std::shared_ptr<rviz_rendering::Shape>> cones_;

    rviz_common::properties::FloatProperty * height_property_;
    rviz_common::properties::ColorProperty * color_property_;
    rviz_common::properties::FloatProperty * alpha_property_;
    rviz_common::properties::IntProperty * buffer_length_property_;
};

}  // namespace displays
}  // namespace rviz_legged_plugins