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
#include <string>

#include "camera_aravis2/stream_statistics.h"

namespace camera_aravis2
{
namespace
{
using Status = diagnostic_msgs::msg::DiagnosticStatus;
using namespace std::chrono_literals;

TEST(StreamStatistics, ReportsIntervalRateAndPreservesTotalsSinceStreamCreation)
{
    StreamCounters previous;
    previous.underruns = 12;
    previous.published = 100;
    StreamCounters current = previous;
    current.completed = 120;
    current.published += 20;
    current.resent = 2;

    const auto report = current.describeInterval(previous, 500ms, true, true);
    EXPECT_EQ(report.level, Status::OK);
    EXPECT_EQ(report.message, "Streaming");
    std::map<std::string, std::string> values;
    for (const auto& entry : report.values)
        values.emplace(entry.key, entry.value);
    EXPECT_DOUBLE_EQ(std::stod(values.at("published_fps")), 40.0);
    EXPECT_EQ(values.at("published_frames"), "120");
    EXPECT_EQ(values.at("buffer_underruns"), "12");
    EXPECT_EQ(values.at("resent_packets"), "2");

    const auto idle = current.describeInterval(current, 1s, false, false);
    EXPECT_EQ(idle.level, Status::OK);
    EXPECT_EQ(idle.message, "Idle");
    for (const auto& entry : idle.values)
    {
        EXPECT_NE(entry.key, "resent_packets");
        EXPECT_NE(entry.key, "missing_packets");
        if (entry.key == "published_fps")
        {
            EXPECT_DOUBLE_EQ(std::stod(entry.value), 0.0);
        }
    }
}

TEST(StreamStatistics, WarnsOnlyForNewLossAndDistinguishesTransportFromProcessing)
{
    const StreamCounters previous;
    for (auto field : {&StreamCounters::failures, &StreamCounters::underruns,
                       &StreamCounters::missing})
    {
        auto current = previous;
        ++(current.*field);
        auto report = current.describeInterval(previous, 1s, true, true);
        EXPECT_EQ(report.level, Status::WARN);
        EXPECT_EQ(report.message, "Transport loss");
        ++current.dropped;
        report = current.describeInterval(previous, 1s, true, true);
        EXPECT_EQ(report.level, Status::WARN);
        EXPECT_EQ(report.message, "Transport loss and processing drops");
        report = current.describeInterval(current, 1s, true, true);
        EXPECT_EQ(report.level, Status::OK);
        EXPECT_EQ(report.message, "Waiting for frames");
    }
    auto current = previous;
    ++current.dropped;
    const auto report = current.describeInterval(previous, 1s, true, false);
    EXPECT_EQ(report.level, Status::WARN);
    EXPECT_EQ(report.message, "Processing drops");
}

TEST(StreamStatistics, DistinguishesSilentContinuousTriggeredFiniteAndIdleAcquisition)
{
    StreamCounters counters;
    const auto report = [&](FrameExpectation mode, bool requested, bool expired)
    {
        return counters.describeInterval(counters, 1s, requested, true, mode, expired);
    };
    EXPECT_EQ(report(FrameExpectation::Continuous, true, true).level, Status::ERROR);
    EXPECT_EQ(report(FrameExpectation::Continuous, true, false).level, Status::OK);
    EXPECT_EQ(report(FrameExpectation::Continuous, false, true).message, "Idle");
    EXPECT_EQ(report(FrameExpectation::Triggered, true, true).message, "Waiting for trigger");
    EXPECT_EQ(report(FrameExpectation::Finite, true, true).level, Status::OK);
    EXPECT_EQ(report(FrameExpectation::Unknown, true, true).level, Status::OK);
}

TEST(StreamStatistics, ReportsTimestampDropsInTheirOwnInterval)
{
    StreamCounters previous;
    auto current = previous;
    ++current.timestamp_drops;
    const auto report = current.describeInterval(previous, 1s, true, true);
    EXPECT_EQ(report.level, Status::WARN);
    EXPECT_EQ(report.message, "Timestamp drops");
    ++current.failures;
    EXPECT_EQ(current.describeInterval(previous, 1s, true, true).message,
              "Transport loss and timestamp drops");
    EXPECT_EQ(current.describeInterval(current, 1s, false, true).level, Status::OK);
}
}  // namespace
}  // namespace camera_aravis2
