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

#include "camera_aravis2/frame_timestamp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace camera_aravis2
{

const char* timestampSourceName(TimestampSource source)
{
    switch (source)
    {
    case TimestampSource::Device: return "device";
    case TimestampSource::DeviceOffset: return "device_offset";
    case TimestampSource::HostReceive: return "host_receive";
    }
    throw std::invalid_argument("Unknown timestamp source");
}

TimestampSource timestampSource(const std::string& mode, bool ptp_enabled)
{
    if (mode == "auto")
        return ptp_enabled ? TimestampSource::Device : TimestampSource::HostReceive;
    for (auto source : {TimestampSource::HostReceive, TimestampSource::Device,
                        TimestampSource::DeviceOffset})
        if (mode == timestampSourceName(source))
            return source;
    throw std::invalid_argument(
      "timestamp_mode must be auto, host_receive, device or device_offset");
}

std::optional<int64_t> timestampOffsetNs(double seconds)
{
    // Match rclcpp's conversion from seconds, checking the range before the cast.
    const double ns = seconds * 1e9;
    constexpr double limit = 0x1p63;
    if (!std::isfinite(ns) || ns < -limit || ns >= limit)
        return std::nullopt;
    return static_cast<int64_t>(ns);
}

std::optional<int64_t> FrameTimestamp::stamp(TimestampSource source, uint64_t device,
                                             uint64_t receive, int64_t offset)
{
    uint64_t base = source == TimestampSource::Device ? device : receive;
    if (source == TimestampSource::DeviceOffset)
    {
        if (device == 0 || receive == 0 || device == last_device_)
            return std::nullopt;

        // Several completed buffers can share a host clock tick. Wait for room to
        // advance without assigning two frames the same aligned timestamp.
        if (last_device_ != 0 && receive == mapped_)
            return std::nullopt;

        if (last_device_ == 0 || device < last_device_ || receive < last_receive_)
            mapped_ = receive;
        else
        {
            // Follow camera intervals, capped at reception. Let the minimum observed
            // host delay rise by up to 100 ppm to track a slower camera oscillator.
            const uint64_t delta = device - last_device_;
            mapped_ += std::min(delta, receive - mapped_);
            mapped_ += std::min(delta / 10000, receive - mapped_);
        }
        last_device_ = device;
        last_receive_ = receive;
        base = mapped_;
    }

    // ROS message seconds are signed 32-bit. Work in integer nanoseconds so rounding
    // at large epoch values cannot obscure small intervals or wrap into a valid stamp.
    constexpr uint64_t max_stamp =
      (static_cast<uint64_t>(std::numeric_limits<int32_t>::max()) + 1) * 1000000000 - 1;
    if (offset < 0)
    {
        const uint64_t magnitude = uint64_t{0} - static_cast<uint64_t>(offset);
        if (base < magnitude)
            return std::nullopt;
        base -= magnitude;
    }
    else
    {
        if (base > max_stamp || static_cast<uint64_t>(offset) > max_stamp - base)
            return std::nullopt;
        base += static_cast<uint64_t>(offset);
    }
    if (base > max_stamp)
        return std::nullopt;
    return static_cast<int64_t>(base);
}

}  // namespace camera_aravis2
