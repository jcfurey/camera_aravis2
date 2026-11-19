// Copyright (c) 2026 camera_aravis2 contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <memory>

#include "camera_aravis2/camera_xml_exporter.h"

namespace camera_aravis2
{

namespace
{
class TestCamera : public CameraXmlExporter
{
  public:
    using CameraXmlExporter::CameraXmlExporter;

    ArvDevice* device() const { return p_device_; }
};
}  // namespace

TEST(ControlLoss, OnlyInvalidatesTheSignallingCamera)
{
    // The two opens create independent Aravis devices entirely in memory.
    arv_disable_interface("GigEVision");
    arv_disable_interface("USB3Vision");
    arv_enable_interface("Fake");
    rclcpp::init(0, nullptr);
    const auto options = rclcpp::NodeOptions().append_parameter_override("guid", "Fake_1");
    auto first = std::make_unique<TestCamera>(options);
    auto sibling = std::make_unique<TestCamera>(options);
    ASSERT_TRUE(first->isInitialized());
    ASSERT_TRUE(sibling->isInitialized());

    auto* device = first->device();
    g_object_ref(device);
    g_signal_emit_by_name(device, "control-lost");
    EXPECT_FALSE(first->isInitialized());
    EXPECT_TRUE(rclcpp::ok());
    EXPECT_TRUE(sibling->isInitialized());

    first.reset();
    // Unloading a component must preserve the process-wide discovery list.
    EXPECT_EQ(arv_get_n_devices(), 1u);
    EXPECT_STREQ(arv_get_device_id(0), "Fake_1");
    // An external device reference can outlive its component.
    g_signal_emit_by_name(device, "control-lost");
    g_object_unref(device);
    EXPECT_GT(arv_device_get_integer_feature_value(sibling->device(), "Width", nullptr), 0);
    sibling.reset();
    rclcpp::shutdown();
}

}  // namespace camera_aravis2
