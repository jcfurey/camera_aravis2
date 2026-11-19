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
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "camera_aravis2/camera_driver.h"
#include "camera_aravis2/frame_io_chunks.h"
#include "camera_aravis2/pixel_range_chunks.h"

namespace camera_aravis2
{
namespace
{
std::string chunkSchema(const std::string& first, const std::string& second)
{
    return "<RegisterDescription>" + first + second + "</RegisterDescription>";
}

std::string integerNode(const std::string& name, const std::string& value)
{
    return "<Integer Name=\"" + name + "\"><Value>" + value + "</Value></Integer>";
}

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

    void exposeLayout()
    {
        declare_parameter<int64_t>("Width", width());
        declare_parameter<std::string>("PixelFormat", "Mono8");
        dynamic_parameters_names_ = {"Width", "PixelFormat"};
    }

    int64_t width() const
    {
        return arv_device_get_integer_feature_value(p_device_, "Width", nullptr);
    }
    size_t bufferSize() const { return streams_[0].p_buffer_pool->getPayloadSize(); }

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

TEST(FrameMetadata, IntegerChunksPreserveBitsAndClearUnavailableValues)
{
    // Fixed GenICam values exercise the Aravis integer parser without a network camera.
    // The parser XML varies to model independently missing fields in successive reads.
    const auto sequence = integerNode("ChunkSequencerSetActive", "9007199254740993");
    const auto lines = integerNode("ChunkLineStatusAll", "-9223372036854775807");
    const auto schema = chunkSchema(sequence, lines);
    std::unique_ptr<ArvGc, decltype(&g_object_unref)> gc(
      arv_gc_new(nullptr, schema.data(), schema.size()), g_object_unref);
    ASSERT_TRUE(gc);
    FrameIoChunks io(gc.get());
    ASSERT_TRUE(io.supported());

    std::unique_ptr<ArvBuffer, decltype(&g_object_unref)> buffer(
      arv_buffer_new_allocate(1), g_object_unref);
    camera_aravis2_msgs::msg::FrameMetadata msg;
    const auto read = [&](const std::string& features)
    {
        std::unique_ptr<ArvChunkParser, decltype(&g_object_unref)> parser(
          arv_chunk_parser_new(features.data(), features.size()), g_object_unref);
        ASSERT_TRUE(parser);
        io.read(parser.get(), buffer.get(), msg);
    };
    read(schema);
    EXPECT_TRUE(msg.sequencer_set_valid);
    EXPECT_EQ(msg.sequencer_set, 9007199254740993ULL);
    EXPECT_TRUE(msg.line_status_valid);
    EXPECT_EQ(msg.line_status, 0x8000000000000001ULL);

    read(chunkSchema(integerNode("ChunkSequencerSetActive", "-1"),
                     integerNode("ChunkLineStatusAll", "0")));
    EXPECT_FALSE(msg.sequencer_set_valid);
    EXPECT_EQ(msg.sequencer_set, 0u);
    EXPECT_TRUE(msg.line_status_valid);
    EXPECT_EQ(msg.line_status, 0u);

    read(chunkSchema(integerNode("ChunkSequencerSetActive", "0"), ""));
    EXPECT_TRUE(msg.sequencer_set_valid);
    EXPECT_EQ(msg.sequencer_set, 0u);
    EXPECT_FALSE(msg.line_status_valid);
    EXPECT_EQ(msg.line_status, 0u);

    read(chunkSchema("", lines));
    EXPECT_FALSE(msg.sequencer_set_valid);
    EXPECT_TRUE(msg.line_status_valid);
    EXPECT_EQ(msg.line_status, 0x8000000000000001ULL);

    io.read(nullptr, buffer.get(), msg);
    EXPECT_FALSE(msg.sequencer_set_valid);
    EXPECT_EQ(msg.sequencer_set, 0u);
    EXPECT_FALSE(msg.line_status_valid);
    EXPECT_EQ(msg.line_status, 0u);
}

TEST(FrameMetadata, PixelRangeRequiresTwoOrderedNonnegativeCountsInTheSameFrame)
{
    const auto xml = [](const std::string& low, const std::string& high)
    {
        return chunkSchema(low.empty() ? "" : integerNode("ChunkPixelDynamicRangeMin", low),
                           high.empty() ? "" : integerNode("ChunkPixelDynamicRangeMax", high));
    };
    const auto schema = xml("0", "255");
    std::unique_ptr<ArvGc, decltype(&g_object_unref)> gc(
      arv_gc_new(nullptr, schema.data(), schema.size()), g_object_unref);
    ASSERT_TRUE(gc);
    PixelRangeChunks range(gc.get());
    ASSERT_TRUE(range.supported());

    std::unique_ptr<ArvBuffer, decltype(&g_object_unref)> buffer(
      arv_buffer_new_allocate(1), g_object_unref);
    camera_aravis2_msgs::msg::FrameMetadata msg;
    const auto check = [&](const std::string& low, const std::string& high, bool valid,
                           uint64_t expected_low, uint64_t expected_high)
    {
        SCOPED_TRACE("min=" + low + ", max=" + high);
        const auto features = xml(low, high);
        std::unique_ptr<ArvChunkParser, decltype(&g_object_unref)> parser(
          arv_chunk_parser_new(features.data(), features.size()), g_object_unref);
        ASSERT_TRUE(parser);
        range.read(parser.get(), buffer.get(), msg);
        EXPECT_EQ(msg.pixel_dynamic_range_valid, valid);
        EXPECT_EQ(msg.pixel_dynamic_range_min, expected_low);
        EXPECT_EQ(msg.pixel_dynamic_range_max, expected_high);
    };
    check("0", "255", true, 0, 255);
    check("16", "", false, 0, 0);
    check("", "4095", false, 0, 0);
    check("8", "8", false, 0, 0);
    check("8", "2", false, 0, 0);
    check("-1", "255", false, 0, 0);
    check("16", "9007199254740993", true, 16, 9007199254740993ULL);
    range.read(nullptr, buffer.get(), msg);
    EXPECT_FALSE(msg.pixel_dynamic_range_valid);
    EXPECT_EQ(msg.pixel_dynamic_range_min, 0u);
    EXPECT_EQ(msg.pixel_dynamic_range_max, 0u);
}

TEST(DynamicFeatures, ResizesTheReceivePoolAndRollsBackRejectedBatches)
{
    arv_disable_interface("GigEVision");
    arv_disable_interface("USB3Vision");
    arv_enable_interface("Fake");
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.append_parameter_override("guid", "Fake_1");
    auto driver = std::make_shared<MemoryCamera>(options);
    ASSERT_TRUE(driver->isInitialized());
    driver->exposeLayout();
    const auto initial = driver->width();
    const auto size = driver->bufferSize();
    using Param = rclcpp::Parameter;
    EXPECT_TRUE(driver->set_parameters_atomically({Param("Width", initial / 2)}).successful);
    EXPECT_EQ(driver->width(), initial / 2);
    EXPECT_EQ(driver->bufferSize(), size / 2);
    EXPECT_FALSE(driver->set_parameters_atomically({
      Param("Width", initial), Param("PixelFormat", "InvalidFormat")}).successful);
    EXPECT_EQ(driver->width(), initial / 2);
    EXPECT_EQ(driver->get_parameter("Width").as_int(), initial / 2);
    EXPECT_EQ(driver->bufferSize(), size / 2);
    EXPECT_TRUE(driver->set_parameters_atomically({Param("Width", initial)}).successful);
    EXPECT_EQ(driver->bufferSize(), size);
    driver.reset();
    rclcpp::shutdown();
}

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
          EXPECT_FALSE(msg.chunks_present);
          EXPECT_FALSE(msg.exposure_valid);
          EXPECT_TRUE(std::isnan(msg.exposure_time));
          EXPECT_TRUE(msg.exposure_unit.empty());
          EXPECT_FALSE(msg.gain_valid);
          EXPECT_TRUE(std::isnan(msg.gain));
          EXPECT_TRUE(msg.gain_unit.empty());
          EXPECT_FALSE(msg.sequencer_set_valid);
          EXPECT_EQ(msg.sequencer_set, 0u);
          EXPECT_FALSE(msg.line_status_valid);
          EXPECT_EQ(msg.line_status, 0u);
          EXPECT_FALSE(msg.pixel_dynamic_range_valid);
          EXPECT_EQ(msg.pixel_dynamic_range_min, 0u);
          EXPECT_EQ(msg.pixel_dynamic_range_max, 0u);
          frames.emplace(stamp, msg.frame_id);
      });
    ASSERT_TRUE(wait([&]() { return driver->acquiring() && frames.size() >= 3; }));
    auto image = reader->create_subscription<sensor_msgs::msg::Image>(prefix + "image_raw", 10,
      [&](const sensor_msgs::msg::Image& msg)
      {
          EXPECT_EQ(msg.header.frame_id, "optical_vis");
          EXPECT_EQ(msg.encoding, "mono8");
          EXPECT_EQ(msg.data.size(), static_cast<size_t>(msg.step) * msg.height);
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
