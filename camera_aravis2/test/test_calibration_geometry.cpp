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

#include "camera_aravis2/calibration_geometry.h"

namespace camera_aravis2
{
namespace
{

class CalibrationTest : public ::testing::Test
{
  protected:
    CalibrationTest()
    {
        source.width = 640;
        source.height = 480;
        // Nonzero skew and stereo translation exercise all affected matrix columns.
        source.k = {500, 10, 320, 0, 450, 240, 0, 0, 1};
        source.p = {500, 10, 320, -60, 0, 450, 240, 12, 0, 0, 1, 0};
        source.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        source.distortion_model = "plumb_bob";
        source.d = {0.1, -0.2, 0.01, -0.02, 0.03};
        header.frame_id = "sensor_optical";
        header.stamp.sec = 123;
        header.stamp.nanosec = 456;
        geometry.x = {640, 2, false};
        geometry.y = {480, 3, false};
        geometry.known = true;
        roi.x = 10;
        roi.y = 20;
        roi.width = 100;
        roi.height = 80;
        geometry.startup_roi = roi;
    }

    sensor_msgs::msg::CameraInfo info() const
    {
        return cameraInfoForFrame(source, options, geometry, roi, header);
    }

    void expectUnknown() const
    {
        sensor_msgs::msg::CameraInfo expected;
        expected.header = header;
        expected.width = roi.width;
        expected.height = roi.height;
        EXPECT_EQ(info(), expected);
    }

    sensor_msgs::msg::CameraInfo source;
    std_msgs::msg::Header header;
    CalibrationOptions options;
    CalibrationGeometry geometry;
    ImageRoi roi;
};

TEST_F(CalibrationTest, PreservesOutputCalibrationAcrossSamplingAndFlips)
{
    source.width = roi.width;
    source.height = roi.height;
    geometry.x.reversed = geometry.y.reversed = true;
    auto expected = source;
    expected.header = header;
    EXPECT_EQ(info(), expected);
}

TEST_F(CalibrationTest, RejectsMismatchedOutputResolutionAndSameSizeShiftedCrop)
{
    expectUnknown();
    source.width = roi.width;
    source.height = roi.height;
    ++roi.x;
    expectUnknown();
}

TEST_F(CalibrationTest, ProjectsThroughSamplingCropAndEitherOrderOfReversal)
{
    options.full_sensor = true;
    options.sampling_offset = {0.5, 1.0};
    // Independently project (0.4, -0.2, 2) through the original K/P.
    for (bool before : {false, true})
    {
        options.reverse_before_roi = before;
        for (int flips = 0; flips < 4; ++flips)
        {
            geometry.x.reversed = (flips & 1) != 0;
            geometry.y.reversed = (flips & 2) != 0;
            const auto result = info();
            const auto projected = [&](double coordinate, int stride, double phase, int extent,
                                       int offset, int length, bool reverse)
            {
                double u = (coordinate - phase) / stride;
                if (reverse && before) u = extent / stride - 1 - u;
                u -= offset;
                if (reverse && !before) u = length - 1 - u;
                return u;
            };
            const double u = projected(419, 2, 0.5, 640, 10, 100, geometry.x.reversed);
            const double v = projected(195, 3, 1.0, 480, 20, 80, geometry.y.reversed);
            EXPECT_DOUBLE_EQ(result.k[0] * .2 - result.k[1] * .1 + result.k[2], u);
            EXPECT_DOUBLE_EQ(-result.k[4] * .1 + result.k[5], v);
            EXPECT_DOUBLE_EQ(result.p[0] * .2 - result.p[1] * .1 + result.p[2] + result.p[3] / 2,
              projected(389, 2, 0.5, 640, 10, 100, geometry.x.reversed));
            EXPECT_DOUBLE_EQ(-result.p[5] * .1 + result.p[6] + result.p[7] / 2,
              projected(201, 3, 1.0, 480, 20, 80, geometry.y.reversed));
            EXPECT_EQ(result.width, 100u);
            EXPECT_EQ(result.height, 80u);
            EXPECT_EQ(result.header, header);
            EXPECT_EQ(result.d, source.d);
            EXPECT_EQ(result.r, source.r);
            EXPECT_EQ(result.distortion_model, source.distortion_model);
        }
    }
}

TEST_F(CalibrationTest, ReadsCropOffsetsInSensorOrSampledPixels)
{
    options.full_sensor = true;
    const auto sampled = info();
    options.roi_in_sensor_pixels = true;
    roi.x *= geometry.x.step;
    roi.y *= geometry.y.step;
    EXPECT_EQ(info(), sampled);
}

TEST_F(CalibrationTest, UsesFreshCalibrationAndRecoversAfterMismatchedFrame)
{
    options.full_sensor = true;
    auto good = info();
    ASSERT_NE(good.k[0], 0.0);
    roi.x = 400;
    expectUnknown();
    roi.x = 10;
    source.k[0] = 222.0;
    EXPECT_DOUBLE_EQ(info().k[0], 111.0);
}

TEST_F(CalibrationTest, RequiresKnownGeometryAndUntransformedSensorCalibration)
{
    options.full_sensor = true;
    geometry.known = false;
    expectUnknown();
    geometry.known = true;
    source.width = roi.width;
    expectUnknown();
    source.width = 640;
    source.roi.x_offset = 1;
    expectUnknown();
    source.roi.x_offset = 0;
    source.binning_y = 2;
    expectUnknown();
    source.binning_y = 0;
    geometry.y.step = 0;
    expectUnknown();
    geometry.y.step = 3;
    options.sampling_offset[0] = 2.0;
    expectUnknown();
}

TEST_F(CalibrationTest, AcceptsExplicitWholeSensorRegion)
{
    options.full_sensor = true;
    const auto expected = info();
    ASSERT_NE(expected.k[0], 0.0);
    source.roi.width = source.width;
    source.roi.height = source.height;
    EXPECT_EQ(info(), expected);
    --source.roi.width;
    expectUnknown();
}

TEST_F(CalibrationTest, KeepsUncalibratedFramesUncalibrated)
{
    source.k[0] = 0;
    expectUnknown();
    options.full_sensor = true;
    expectUnknown();
}

}  // namespace
}  // namespace camera_aravis2
