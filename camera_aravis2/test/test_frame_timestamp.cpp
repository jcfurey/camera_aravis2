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

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "camera_aravis2/frame_timestamp.h"

namespace camera_aravis2
{
namespace
{

TEST(FrameTimestamp, AutoFollowsPtpAndExplicitModesTakePrecedence)
{
    EXPECT_EQ(timestampSource("auto", false), TimestampSource::HostReceive);
    EXPECT_EQ(timestampSource("auto", true), TimestampSource::Device);
    for (bool ptp : {false, true})
    {
        EXPECT_EQ(timestampSource("host_receive", ptp), TimestampSource::HostReceive);
        EXPECT_EQ(timestampSource("device", ptp), TimestampSource::Device);
        EXPECT_EQ(timestampSource("device_offset", ptp), TimestampSource::DeviceOffset);
    }
    EXPECT_THROW(timestampSource("typo", false), std::invalid_argument);
}

TEST(FrameTimestamp, DirectModesPreserveTheSourceAndApplyTheLiveOffsetOnce)
{
    FrameTimestamp clock;
    EXPECT_EQ(clock.stamp(TimestampSource::HostReceive, 20, 5000, -5), 4995);
    EXPECT_EQ(clock.stamp(TimestampSource::Device, 20, 5000, 7), 27);
    EXPECT_EQ(clock.stamp(TimestampSource::Device, 19, 5100, 0), 19);
}

TEST(FrameTimestamp, AlignedDeviceTimeFollowsIntervalsAndTracksDrift)
{
    FrameTimestamp clock;
    const auto source = TimestampSource::DeviceOffset;
    // A raw clock close to UINT64_MAX need not share the host epoch.
    constexpr uint64_t device = UINT64_MAX - 1000000;
    constexpr uint64_t receive = 1795000000000000000ULL;
    EXPECT_EQ(clock.stamp(source, device, receive, 0), receive);
    EXPECT_EQ(clock.stamp(source, device + 10000, receive + 30000, 0), receive + 10001);
    EXPECT_EQ(clock.stamp(source, device + 20000, receive + 30001, 4), receive + 20006);
    // A shorter path lowers the offset estimate immediately.
    EXPECT_EQ(clock.stamp(source, device + 40000, receive + 32000, 0), receive + 32000);
}

TEST(FrameTimestamp, InvalidSamplesAndClockRestartsAreIsolatedPerStream)
{
    FrameTimestamp first, second;
    const auto source = TimestampSource::DeviceOffset;
    EXPECT_FALSE(first.stamp(source, 0, 100, 0));
    EXPECT_FALSE(first.stamp(source, 1, 0, 0));
    EXPECT_EQ(first.stamp(source, 300, 3000, 0), 3000);
    EXPECT_FALSE(first.stamp(source, 300, 3001, 0));
    EXPECT_FALSE(first.stamp(source, 301, 3000, 0));
    EXPECT_EQ(second.stamp(source, 300, 9000, 0), 9000);
    EXPECT_EQ(first.stamp(source, 305, 3500, 0), 3005);
    EXPECT_EQ(first.stamp(source, 2, 3600, 0), 3600);
    EXPECT_EQ(first.stamp(source, 3, 2000, 0), 2000);
}

TEST(FrameTimestamp, ChecksOffsetAndMessageBoundsBeforeAddingOrNarrowing)
{
    FrameTimestamp clock;
    const auto source = TimestampSource::Device;
    constexpr int64_t max_stamp = (int64_t{INT32_MAX} + 1) * 1000000000 - 1;
    EXPECT_EQ(clock.stamp(source, max_stamp, 0, 0), max_stamp);
    EXPECT_FALSE(clock.stamp(source, max_stamp, 0, 1));
    EXPECT_FALSE(clock.stamp(source, 0, 0, -1));
    EXPECT_FALSE(clock.stamp(source, UINT64_MAX, 0, INT64_MIN));
    EXPECT_FALSE(clock.stamp(source, UINT64_MAX, 0, INT64_MAX));
    EXPECT_EQ(clock.stamp(source, uint64_t{1} << 63, 0, INT64_MIN), 0);
    EXPECT_EQ(clock.stamp(source, max_stamp + 1, 0, -1), max_stamp);
    EXPECT_EQ(timestampOffsetNs(-0.25), -250000000);
    EXPECT_FALSE(timestampOffsetNs(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(timestampOffsetNs(std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(timestampOffsetNs(1e20));
}

}  // namespace
}  // namespace camera_aravis2
