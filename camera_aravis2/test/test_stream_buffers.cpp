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

#include <algorithm>
#include <memory>

#include "camera_aravis2/image_buffer_pool.h"

namespace camera_aravis2
{

TEST(StreamBuffers, PublishesOnlyPixelsAndPreservesAravisStorage)
{
    std::unique_ptr<ArvDevice, decltype(&g_object_unref)> device(
      arv_fake_device_new("pixel-test", nullptr), g_object_unref);
    ASSERT_NE(device, nullptr);
    auto* camera = arv_fake_device_get_fake_camera(ARV_FAKE_DEVICE(device.get()));
    std::unique_ptr<ArvStream, decltype(&g_object_unref)> stream(
      arv_device_create_stream(device.get(), nullptr, nullptr, nullptr), g_object_unref);
    ASSERT_NE(stream, nullptr);
    const size_t payload = arv_fake_camera_get_payload(camera);
    ImageBufferPool pool(rclcpp::get_logger("pixel_test"), stream.get(), payload, 0);
    for (const size_t extra : {0u, 32u})
    {
        auto received = std::make_shared<sensor_msgs::msg::Image>();
        received->data.resize(payload + extra, 0xa5);
        std::unique_ptr<ArvBuffer, decltype(&g_object_unref)> buffer(
          arv_buffer_new(received->data.size(), received->data.data()), g_object_unref);
        arv_fake_camera_fill_buffer(camera, buffer.get(), nullptr);
        auto pixels = pool.imagePixels(buffer.get(), received);
        ASSERT_NE(pixels, nullptr);
        EXPECT_EQ(pixels == received, extra == 0);
        ASSERT_EQ(pixels->data.size(), payload);
        EXPECT_EQ(pixels->step, static_cast<uint32_t>(arv_buffer_get_image_width(buffer.get())));
        EXPECT_TRUE(std::equal(pixels->data.begin(), pixels->data.end(), received->data.begin()));
        EXPECT_EQ(received->data.size(), payload + extra);
        EXPECT_EQ(received->data.data(), arv_buffer_get_data(buffer.get(), nullptr));
        if (extra)
        {
            EXPECT_EQ(received->data.back(), 0xa5);
        }
    }
}

TEST(StreamBuffers, RecyclesNewestFrameAtLimitAndResumesWhenConsumerReleasesBuffer)
{
    std::unique_ptr<ArvDevice, decltype(&g_object_unref)> device(
      arv_fake_device_new("buffer-test", nullptr), g_object_unref);
    ASSERT_NE(device, nullptr);
    std::unique_ptr<ArvStream, decltype(&g_object_unref)> stream(
      arv_device_create_stream(device.get(), nullptr, nullptr, nullptr), g_object_unref);
    ASSERT_NE(stream, nullptr);
    const auto payload = arv_device_get_integer_feature_value(device.get(), "PayloadSize", nullptr);
    ASSERT_GT(payload, 0);
    ImageBufferPool pool(rclcpp::get_logger("pool_test"), stream.get(), payload, 2);
    arv_device_execute_command(device.get(), "AcquisitionStart", nullptr);
    const auto pop = [&]() { return arv_stream_timeout_pop_buffer(stream.get(), 2000000); };

    // Hold both initial buffers outside the receiver. Accepting one grows the pool.
    auto* first = pop();
    auto* second = pop();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    constexpr size_t limit = 3;
    auto processing = pool.acquireFrame(first, limit);
    ASSERT_NE(processing, nullptr);
    ASSERT_EQ(pool.getAllocatedSize(), limit);
    auto* third = pop();
    ASSERT_NE(third, nullptr);

    // Keep the worker stalled. The receiver keeps filling the same recycled buffer.
    for (int i = 0; i < 5; ++i)
    {
        EXPECT_EQ(pool.acquireFrame(second, limit), nullptr);
        auto* recycled = pop();
        ASSERT_EQ(recycled, second);
        EXPECT_EQ(arv_buffer_get_status(recycled), ARV_BUFFER_STATUS_SUCCESS);
        EXPECT_EQ(pool.getAllocatedSize(), limit);
    }

    // Return the retained buffers to the receiver and resume processing.
    processing.reset();
    arv_stream_push_buffer(stream.get(), third);
    bool resumed = false;
    for (int i = 0; i < 5 && !resumed; ++i)
    {
        resumed = static_cast<bool>(pool.acquireFrame(second, limit));
        if (!resumed)
        {
            second = pop();
            ASSERT_NE(second, nullptr);
        }
    }
    EXPECT_TRUE(resumed);
    EXPECT_EQ(pool.getAllocatedSize(), limit);
    arv_device_execute_command(device.get(), "AcquisitionStop", nullptr);
    stream.reset();
}

}  // namespace camera_aravis2
