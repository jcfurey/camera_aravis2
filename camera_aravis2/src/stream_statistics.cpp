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

#include "camera_aravis2/stream_statistics.h"

#include <string>

#include "camera_aravis2/camera_driver.h"

namespace camera_aravis2
{

namespace
{
void addText(diagnostic_msgs::msg::DiagnosticStatus& status, const char* key,
             const std::string& value)
{
    auto& entry = status.values.emplace_back();
    entry.key = key;
    entry.value = value;
}

template <typename T>
void addValue(diagnostic_msgs::msg::DiagnosticStatus& status, const char* key, T value)
{
    addText(status, key, std::to_string(value));
}
}  // namespace

diagnostic_msgs::msg::DiagnosticStatus StreamCounters::describeInterval(
  const StreamCounters& previous, std::chrono::duration<double> interval,
  bool acquisition_requested, bool is_gige, FrameExpectation expectation, bool stalled) const
{
    using Status = diagnostic_msgs::msg::DiagnosticStatus;
    Status status;
    const auto published_delta = published - previous.published;
    addValue(status, "completed_buffers", completed);
    addValue(status, "incomplete_buffers", failures);
    addValue(status, "buffer_underruns", underruns);
    if (is_gige)
    {
        addValue(status, "resent_packets", resent);
        addValue(status, "missing_packets", missing);
    }
    addValue(status, "published_frames", published);
    const double fps = interval.count() > 0 ? published_delta / interval.count() : 0.0;
    addValue(status, "published_fps", fps);
    addValue(status, "dropped_frames", dropped);
    addValue(status, "timestamp_drops", timestamp_drops);

    const bool transport_loss = failures > previous.failures ||
      underruns > previous.underruns || missing > previous.missing;
    const bool processing_loss = dropped > previous.dropped;
    if (transport_loss || processing_loss)
    {
        status.level = Status::WARN;
        if (transport_loss && processing_loss)
            status.message = "Transport loss and processing drops";
        else
            status.message = transport_loss ? "Transport loss" : "Processing drops";
    }
    else if (!acquisition_requested)
        status.message = "Idle";
    else if (!published_delta && expectation == FrameExpectation::Triggered)
        status.message = "Waiting for trigger";
    else if (!published_delta && expectation == FrameExpectation::Finite)
        status.message = "Finite acquisition";
    else if (!published_delta && expectation == FrameExpectation::Continuous && stalled)
    {
        status.level = Status::ERROR;
        status.message = "Free-running stream stalled";
    }
    else
        status.message = published_delta ? "Streaming" : "Waiting for frames";
    if (timestamp_drops > previous.timestamp_drops)
    {
        status.message = status.level == Status::WARN ?
          status.message + " and timestamp drops" : "Timestamp drops";
        status.level = Status::WARN;
    }
    return status;
}

StreamCounters CameraDriver::readStreamCounters(const Stream& stream)
{
    StreamCounters counters;
    arv_stream_get_statistics(stream.p_arv_stream, &counters.completed,
                              &counters.failures, &counters.underruns);
    if (ARV_IS_GV_STREAM(stream.p_arv_stream))
        arv_gv_stream_get_statistics(ARV_GV_STREAM(stream.p_arv_stream),
                                     &counters.resent, &counters.missing);
    counters.published = stream.published_frames.load();
    counters.dropped = stream.dropped_frames.load();
    counters.timestamp_drops = stream.timestamp_drops.load();
    return counters;
}

void CameraDriver::publishStreamDiagnostics()
{
    using Status = diagnostic_msgs::msg::DiagnosticStatus;
    diagnostic_msgs::msg::DiagnosticArray msg;
    {
        std::lock_guard<std::mutex> lock(acquisition_mutex_);
        const bool unavailable = cameraUnavailable();
        if (!is_initialized_ && !unavailable)
            return;

        msg.header.stamp = now();
        const auto sample = std::chrono::steady_clock::now();
        const std::chrono::duration<double> interval = sample - last_stream_sample_;
        last_stream_sample_ = sample;
        if (streams_.empty() && unavailable)
        {
            auto& status = msg.status.emplace_back();
            status.level = Status::ERROR;
            status.message = "Camera unavailable";
            status.name = get_fully_qualified_name();
            status.hardware_id = guid_;
        }
        for (auto& stream : streams_)
        {
            auto& status = msg.status.emplace_back();
            if (unavailable)
            {
                status.level = Status::ERROR;
                status.message = "Camera control lost";
            }
            else if (!stream.p_arv_stream)
            {
                status.level = Status::ERROR;
                status.message = "Stream unavailable";
            }
            else
            {
                const auto counters = readStreamCounters(stream);
                if (counters.completed != stream.previous_counters.completed)
                    stream.last_frame_activity = sample;
                const bool stalled = stall_timeout_.count() > 0 &&
                  sample - stream.last_frame_activity >= stall_timeout_;
                status = counters.describeInterval(stream.previous_counters, interval,
                                                   current_num_subscribers_ > 0,
                                                   ARV_IS_GV_STREAM(stream.p_arv_stream),
                                                   stream.frame_expectation, stalled);
                addText(status, "timestamp_source", timestampSourceName(timestamp_source_));
                if (ptp_lock_)
                {
                    std::lock_guard<std::mutex> clock_guard(ptp_mutex_);
                    const auto& ptp = ptp_lock_->sample();
                    const bool locked = ptp_lock_->locked(PtpLock::Clock::now());
                    addText(status, "ptp_status", ptp.status);
                    addText(status, "ptp_locked", locked ? "true" : "false");
                    addText(status, "ptp_servo_status", ptp.servo_status.value_or("unavailable"));
                    addText(status, "ptp_offset_ns",
                      ptp.offset_ns ? std::to_string(*ptp.offset_ns) : "unavailable");
                    if (timestamp_source_ == TimestampSource::Ptp && !locked &&
                        current_num_subscribers_ > 0 && status.level != Status::ERROR)
                    {
                        status.message = status.level == Status::WARN ?
                          status.message + " (waiting for PTP slave lock)" :
                          "Waiting for PTP slave lock";
                        status.level = Status::WARN;
                    }
                }
                addValue(status, "processing_queue_depth", stream.buffer_queue.size());
                addValue(status, "buffers_allocated", stream.p_buffer_pool->getAllocatedSize());
                addValue(status, "buffer_limit", max_stream_buffers_);
                stream.previous_counters = counters;
            }
            status.name = std::string(get_fully_qualified_name()) + ": " + stream.name;
            status.hardware_id = hardware_id_;
        }
    }
    stream_diagnostic_pub_->publish(msg);
}

void CameraDriver::printStreamStatistics() const
{
    for (const auto& stream : streams_)
    {
        if (!stream.p_arv_stream)
            continue;
        const auto counters = readStreamCounters(stream);
        RCLCPP_INFO_STREAM(logger_, "Statistics for " << stream.name
          << ": completed=" << counters.completed << " failures=" << counters.failures
          << " underruns=" << counters.underruns << " published=" << counters.published
          << " dropped=" << counters.dropped << " timestamp_drops=" << counters.timestamp_drops);
        if (ARV_IS_GV_STREAM(stream.p_arv_stream))
            RCLCPP_INFO_STREAM(logger_, "Resent packets=" << counters.resent
              << " missing packets=" << counters.missing);
    }
}

}  // namespace camera_aravis2
