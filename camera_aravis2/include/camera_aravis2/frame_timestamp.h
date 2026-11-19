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

#ifndef CAMERA_ARAVIS2__FRAME_TIMESTAMP_H_
#define CAMERA_ARAVIS2__FRAME_TIMESTAMP_H_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace camera_aravis2
{

enum class TimestampSource { HostReceive, Device, DeviceOffset, Ptp };
TimestampSource timestampSource(const std::string& mode, bool ptp_enabled);
const char* timestampSourceName(TimestampSource source);
std::optional<int64_t> timestampOffsetNs(double seconds);

struct PtpSample
{
    std::string status = "Unknown";
    std::optional<std::string> servo_status;
    std::optional<int64_t> offset_ns;
};

// Samples use steady time for expiry and host reception time to exclude queued
// buffers received before lock was confirmed.
class PtpLock
{
  public:
    using Clock = std::chrono::steady_clock;
    PtpLock(std::chrono::seconds window, int64_t max_offset_ns);
    void observe(PtpSample sample, Clock::time_point poll_time, uint64_t host_ns);
    bool locked(Clock::time_point now) const;
    bool covers(uint64_t receive_ns, Clock::time_point now) const;
    const PtpSample& sample() const { return sample_; }

  private:
    std::chrono::seconds window_;
    int64_t max_offset_ns_;
    PtpSample sample_;
    Clock::time_point last_poll_{};
    std::optional<Clock::time_point> good_since_;
    std::optional<uint64_t> eligible_from_ns_;
};

// One instance per ordered stream. All clock inputs and results are nanoseconds.
class FrameTimestamp
{
  public:
    std::optional<int64_t> stamp(TimestampSource source, uint64_t device, uint64_t receive,
                                 int64_t offset, uint64_t max_host_skew_ns = 1000000000);

  private:
    uint64_t last_device_ = 0;
    uint64_t last_receive_ = 0;
    uint64_t mapped_ = 0;
    std::optional<uint64_t> last_ptp_stamp_;
};

}  // namespace camera_aravis2

#endif  // CAMERA_ARAVIS2__FRAME_TIMESTAMP_H_
