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
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <memory>
#include <vector>

#include "camera_aravis2/conversion_utils.h"

namespace camera_aravis2
{
TEST(PackedPixels, BothYuv422LayoutsDecodeRed)
{
    for (const bool yuy2 : {true, false})
    {
        auto image = std::make_shared<sensor_msgs::msg::Image>();
        image->width = 2;
        image->height = 1;
        image->step = 4;
        image->data = yuy2 ? std::vector<uint8_t>{82, 90, 82, 240} :
                             std::vector<uint8_t>{90, 82, 240, 82};
        sensor_msgs::msg::Image::SharedPtr out;
        CONVERSIONS_DICTIONARY.at(yuy2 ? "YUV422_8" : "YUV422_8_UYVY")(image, out);
        ASSERT_TRUE(out);
        const auto rgb = cv_bridge::toCvCopy(out, "rgb8");
        for (int x = 0; x < 2; ++x)
        {
            const auto pixel = rgb->image.at<cv::Vec3b>(0, x);
            EXPECT_GE(pixel[0], 250);
            EXPECT_LE(pixel[1], 3);
            EXPECT_LE(pixel[2], 3);
        }
    }
}

TEST(PackedPixels, LegacyTenBitPairsRetainEveryCodeInBothPositions)
{
    auto image = std::make_shared<sensor_msgs::msg::Image>();
    image->width = 2048;
    image->height = 1;
    image->step = 3072;
    for (unsigned b = 0; b < 1024; ++b)
    {
        const unsigned a = 1023 - b;
        image->data.push_back(a >> 2);
        image->data.push_back((a & 3) | ((b & 3) << 4));
        image->data.push_back(b >> 2);
    }
    for (const char* format : {"Mono10Packed", "BayerRG10Packed", "BayerBG10Packed",
                               "BayerGR10Packed", "BayerGB10Packed"})
    {
        SCOPED_TRACE(format);
        sensor_msgs::msg::Image::SharedPtr out;
        CONVERSIONS_DICTIONARY.at(format)(image, out);
        ASSERT_NE(out, nullptr);
        ASSERT_EQ(out->step, 4096u);
        ASSERT_EQ(out->data.size(), out->step);
        for (unsigned b = 0; b < 1024; ++b)
        {
            const auto* p = out->data.data() + 4 * b;
            ASSERT_EQ(p[0] | (p[1] << 8), (1023 - b) << 6);
            ASSERT_EQ(p[2] | (p[3] << 8), b << 6);
        }
    }
}
}  // namespace camera_aravis2
