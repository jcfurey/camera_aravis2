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

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "camera_aravis2/camera_driver.h"

namespace camera_aravis2
{
namespace
{
class MemoryCamera : public CameraDriver
{
  public:
    explicit MemoryCamera(const rclcpp::NodeOptions& options) :
      CameraDriver("metadata_camera", options)
    {
        setupParameters();
        if (discoverAndOpenCameraDevice() && setupCameraStreamStructs() &&
            setImageFormatControlSettings())
        {
            is_spawning_ = true;
            spawnCameraStreams();
        }
    }

    ~MemoryCamera() override { stopCameraStreams(); }

    bool acquiring() const
    {
        auto* camera = arv_fake_device_get_fake_camera(ARV_FAKE_DEVICE(p_device_));
        return arv_fake_camera_get_acquisition_status(camera) != 0;
    }

  private:
    int discoverNumberOfStreams() override { return 1; }
    bool setTechSpecificTlControlSettings() override { return true; }
    void tuneArvStream(ArvStream*) const override {}
    void postFrameProcessingCallback(uint) override {}
};
}  // namespace

TEST(FrameMetadata, MetadataOnlyAcquisitionAndMatchingImageHeaders)
{
    using namespace std::chrono_literals;
    using Metadata = camera_aravis2_msgs::msg::FrameMetadata;

    // Exercise the shared GV/UV publishing path using only the Aravis in-memory device.
    arv_disable_interface("GigEVision");
    arv_disable_interface("USB3Vision");
    arv_enable_interface("Fake");
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto reader = std::make_shared<rclcpp::Node>("metadata_reader", options);
    options.append_parameter_override("guid", "Fake_1");
    options.append_parameter_override("frame_id", "optical");
    options.append_parameter_override("stream_names", std::vector<std::string>{"vis"});
    options.append_parameter_override("timestamp_offset", 1.25);
    auto driver = std::make_shared<MemoryCamera>(options);
    ASSERT_TRUE(driver->isInitialized());
#ifdef WITH_MATCHED_EVENTS
    EXPECT_FALSE(driver->acquiring());
#endif

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(driver);
    executor.add_node(reader);
    const auto wait = [&](const auto& condition)
    {
        const auto end = std::chrono::steady_clock::now() + 5s;
        while (!condition() && std::chrono::steady_clock::now() < end)
            executor.spin_once(50ms);
        return condition();
    };

    std::map<int64_t, uint64_t> frames;
    std::set<int64_t> images, infos;
    const std::string prefix = "/metadata_camera/vis/";
    auto metadata = reader->create_subscription<Metadata>(prefix + "frame_metadata", 10,
      [&](const Metadata& msg)
      {
          const auto stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
          EXPECT_EQ(msg.header.frame_id, "optical_vis");
          EXPECT_EQ(msg.hardware_id, "Aravis-Fake-1");
          EXPECT_EQ(msg.pixel_format, ARV_PIXEL_FORMAT_MONO_8);
          EXPECT_EQ(stamp, static_cast<int64_t>(msg.host_receive_timestamp_ns) + 1250000000);
          EXPECT_GT(msg.device_timestamp_ns, 0u);
          EXPECT_GT(msg.frame_id, 0u);
          frames.emplace(stamp, msg.frame_id);
      });
    ASSERT_TRUE(wait([&]() { return driver->acquiring() && frames.size() >= 3; }));
    auto image = reader->create_subscription<sensor_msgs::msg::Image>(prefix + "image_raw", 10,
      [&](const sensor_msgs::msg::Image& msg)
      {
          EXPECT_EQ(msg.header.frame_id, "optical_vis");
          EXPECT_EQ(msg.encoding, "mono8");
          images.insert(rclcpp::Time(msg.header.stamp).nanoseconds());
      });
    auto info = reader->create_subscription<sensor_msgs::msg::CameraInfo>(
      prefix + "camera_info", 10,
      [&](const sensor_msgs::msg::CameraInfo& msg)
      {
          EXPECT_EQ(msg.header.frame_id, "optical_vis");
          infos.insert(rclcpp::Time(msg.header.stamp).nanoseconds());
      });
    ASSERT_TRUE(wait([&]()
    {
        std::set<uint64_t> matched;
        for (const auto& [stamp, id] : frames)
            if (images.count(stamp) && infos.count(stamp))
                matched.insert(id);
        return matched.size() >= 5;
    }));

#ifdef WITH_MATCHED_EVENTS
    image.reset();
    info.reset();
    metadata.reset();
    EXPECT_TRUE(wait([&]() { return !driver->acquiring(); }));
#endif
    executor.remove_node(driver);
    driver.reset();
    rclcpp::shutdown();
}

}  // namespace camera_aravis2
