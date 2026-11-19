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
#include <utility>

namespace camera_aravis2
{

const char* timestampSourceName(TimestampSource source)
{
    switch (source)
    {
    case TimestampSource::Ptp: return "ptp";
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
                        TimestampSource::DeviceOffset, TimestampSource::Ptp})
        if (mode == timestampSourceName(source))
            return source;
    throw std::invalid_argument(
      "timestamp_mode must be auto, host_receive, device, device_offset or ptp");
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

namespace
{
constexpr auto max_poll_gap = std::chrono::seconds(2);
}

PtpLock::PtpLock(std::chrono::seconds window, int64_t max_offset_ns) :
  window_(window), max_offset_ns_(max_offset_ns)
{
    if (window.count() < 0 || max_offset_ns < 0)
        throw std::invalid_argument("PTP lock window and offset tolerance must be nonnegative");
}

void PtpLock::observe(PtpSample sample, Clock::time_point poll_time, uint64_t host_ns)
{
    sample_ = std::move(sample);
    const bool good = sample_.status == "Slave" && sample_.offset_ns &&
      *sample_.offset_ns >= -max_offset_ns_ && *sample_.offset_ns <= max_offset_ns_ &&
      (!sample_.servo_status || *sample_.servo_status == "Locked");
    if (!good || poll_time - last_poll_ > max_poll_gap)
    {
        good_since_.reset();
        eligible_from_ns_.reset();
    }
    last_poll_ = poll_time;
    if (!good)
        return;
    if (!good_since_)
        good_since_ = poll_time;
    if (!eligible_from_ns_ &&
        std::chrono::duration_cast<std::chrono::seconds>(poll_time - *good_since_) >= window_)
        eligible_from_ns_ = host_ns;
}

bool PtpLock::locked(Clock::time_point now) const
{
    return eligible_from_ns_.has_value() && now - last_poll_ <= max_poll_gap;
}

bool PtpLock::covers(uint64_t receive_ns, Clock::time_point now) const
{
    return locked(now) && receive_ns >= *eligible_from_ns_;
}

std::optional<int64_t> FrameTimestamp::stamp(TimestampSource source, uint64_t device,
                                             uint64_t receive, int64_t offset,
                                             uint64_t max_host_skew_ns)
{
    const bool ptp = source == TimestampSource::Ptp;
    if (ptp && (device == 0 || receive == 0))
        return std::nullopt;
    uint64_t base = (source == TimestampSource::Device || ptp) ? device : receive;
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
    if (ptp)
    {
        const uint64_t skew = base > receive ? base - receive : receive - base;
        if (skew > max_host_skew_ns || (last_ptp_stamp_ && base <= *last_ptp_stamp_))
            return std::nullopt;
        last_ptp_stamp_ = base;
    }
    return static_cast<int64_t>(base);
}

}  // namespace camera_aravis2
