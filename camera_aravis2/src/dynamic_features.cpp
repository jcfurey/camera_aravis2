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

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "camera_aravis2/camera_driver.h"

namespace camera_aravis2
{
namespace
{
using Region = std::array<gint, 4>;

std::optional<Region> readRegion(ArvCamera* camera)
{
    Region region{};
    GuardedGError err;
    arv_camera_get_region(camera, &region[0], &region[1], &region[2], &region[3], err.ref());
    if (err)
        return std::nullopt;
    return region;
}

bool samplingFeature(const std::string& name)
{
    return name == "SensorShutterMode" || name == "SensorDigitizationTaps" ||
      name == "SensorReadoutMode" || name.rfind("Reverse", 0) == 0 ||
      name.rfind("Binning", 0) == 0 || name.rfind("Decimation", 0) == 0 ||
      name.rfind("Region", 0) == 0 || name.rfind("MultipleROI", 0) == 0;
}

bool needsStoppedStream(const rclcpp::Parameter& param)
{
    const auto& name = param.get_name();
    return samplingFeature(name) || name == "PixelFormat" || name == "Width" ||
      name == "Height" || name.rfind("Offset", 0) == 0 || name.rfind("Center", 0) == 0 ||
      name.rfind("Chunk", 0) == 0 || name.rfind("Trigger", 0) == 0 ||
      name == "AcquisitionMode" || name == "SourceSelector";
}

bool matches(const rclcpp::Parameter& actual, const rclcpp::Parameter& requested)
{
    if (requested.get_type() == rclcpp::PARAMETER_DOUBLE)
    {
        const double value = requested.as_double();
        return std::fabs(actual.as_double() - value) <= 1e-6 * std::max(1.0, std::fabs(value));
    }
    return actual.get_parameter_value() == requested.get_parameter_value();
}
}  // namespace

FrameExpectation CameraDriver::readFrameExpectation() const
{
    std::string mode;
    if (!getFeatureValue("AcquisitionMode", mode))
        return FrameExpectation::Unknown;
    if (mode != "Continuous")
        return FrameExpectation::Finite;
    if (!arv_device_get_feature(p_device_, "TriggerMode"))
        return FrameExpectation::Continuous;

    const auto readMode = [&]()
    {
        std::string trigger;
        if (!getFeatureValue("TriggerMode", trigger))
            return FrameExpectation::Unknown;
        return trigger == "Off" ? FrameExpectation::Continuous : FrameExpectation::Triggered;
    };
    if (!arv_device_get_feature(p_device_, "TriggerSelector"))
        return readMode();

    // TriggerMode is banked. An inactive selected bank can hide an armed FrameStart trigger.
    std::string original;
    if (!getFeatureValue("TriggerSelector", original))
        return FrameExpectation::Unknown;
    auto result = readMode();
    if (result == FrameExpectation::Triggered)
        return result;
    guint count = 0;
    GuardedGError err;
    const char** names = arv_camera_dup_available_triggers(p_camera_, &count, err.ref());
    if (err || !names)
        return FrameExpectation::Unknown;
    bool changed = false;
    for (guint i = 0; i < count; ++i)
    {
        if (names[i] == original)
            continue;
        changed = true;
        if (!setFeatureValue("TriggerSelector", std::string(names[i])))
        {
            result = FrameExpectation::Unknown;
            continue;
        }
        auto bank = readMode();
        if (bank != FrameExpectation::Continuous)
            result = bank;
        if (bank == FrameExpectation::Triggered)
            break;
    }
    g_free(names);
    if (changed && !setFeatureValue("TriggerSelector", original))
        return FrameExpectation::Unknown;
    return result;
}

bool CameraDriver::readDynamicFeature(const rclcpp::Parameter& spec,
                                      rclcpp::Parameter& current) const
{
    const auto read = [&](auto value)
    {
        if (!getFeatureValue(spec.get_name(), value))
            return false;
        current = rclcpp::Parameter(spec.get_name(), value);
        return true;
    };
    switch (spec.get_type())
    {
    case rclcpp::PARAMETER_STRING: return read(std::string());
    case rclcpp::PARAMETER_BOOL: return read(false);
    case rclcpp::PARAMETER_INTEGER: return read(int64_t{});
    case rclcpp::PARAMETER_DOUBLE: return read(0.0);
    default: return false;
    }
}

bool CameraDriver::writeDynamicFeature(const rclcpp::Parameter& value) const
{
    const auto& name = value.get_name();
    switch (value.get_type())
    {
    case rclcpp::PARAMETER_STRING: return setFeatureValue(name, value.as_string());
    case rclcpp::PARAMETER_BOOL: return setFeatureValue(name, value.as_bool());
    case rclcpp::PARAMETER_INTEGER:
    {
        GuardedGError err;
        gint64 low = 0, high = 0;
        arv_device_get_integer_feature_bounds(p_device_, name.c_str(), &low, &high, err.ref());
        const auto requested = value.as_int();
        if (err || requested < low || requested > high)
            return false;
        const gint64 step = arv_device_get_integer_feature_increment(p_device_, name.c_str(),
                                                                     err.ref());
        if (!err && step > 1 && (static_cast<uint64_t>(requested) - low) % step != 0)
            return false;
        return setFeatureValue(name, requested);
    }
    case rclcpp::PARAMETER_DOUBLE: return setFeatureValue(name, value.as_double());
    default: return false;
    }
}

bool CameraDriver::pauseStream(Stream& stream)
{
    arv_stream_set_emit_signals(stream.p_arv_stream, FALSE);
    stream.is_buffer_processed = false;
    // A transport plugin may be publishing while waiting for the ROS parameter lock held by
    // this callback. Bound the wait before touching the hardware configuration.
    {
        std::unique_lock<std::timed_mutex> lock(stream.processing_mutex, std::defer_lock);
        if (!lock.try_lock_for(std::chrono::seconds(1)))
        {
            stream.is_buffer_processed = true;
            arv_stream_set_emit_signals(stream.p_arv_stream, TRUE);
            return false;
        }
    }
    stopBufferWorker(stream);
    // The worker has returned every loan. Aravis can now discard both transport queues.
    arv_stream_stop_thread(stream.p_arv_stream, TRUE);
    return true;
}

bool CameraDriver::resumeStream(Stream& stream, bool acquiring)
{
    std::string format;
    GuardedGError err;
    const auto size = arv_camera_get_payload(p_camera_, err.ref());
    if (err || size == 0 || !getFeatureValue("PixelFormat", format))
        return false;
    const auto converter = CONVERSIONS_DICTIONARY.find(format);
    if (converter == CONVERSIONS_DICTIONARY.end() && format != stream.sensor.pixel_format)
        return false;

    const auto code = arv_camera_get_pixel_format(p_camera_, err.ref());
    if (err || !code)
        return false;
    stream.pixel_format_code = code;
    stream.sensor.n_bits_pixel = ARV_PIXEL_FORMAT_BIT_PER_PIXEL(code);
    stream.sensor.pixel_format = format;
    stream.cvt_pixel_format = converter == CONVERSIONS_DICTIONARY.end() ?
      ConversionFunction{} : converter->second;
    if (calibration_options_.full_sensor)
    {
        stream.calibration_geometry = {};
        readCalibrationGeometry(stream);
    }
    stream.frame_expectation = readFrameExpectation();
    startBufferWorker(0, size);
    arv_stream_start_thread(stream.p_arv_stream);
    if (acquiring && !executeCommand("AcquisitionStart"))
    {
        executeCommand("AcquisitionStop");
        stopBufferWorker(stream);
        arv_stream_stop_thread(stream.p_arv_stream, TRUE);
        return false;
    }
    arv_stream_set_emit_signals(stream.p_arv_stream, TRUE);
    stream.last_frame_activity = std::chrono::steady_clock::now();
    return true;
}

bool CameraDriver::applyDynamicFeatures(const std::vector<rclcpp::Parameter>& features,
                                       std::string& reason)
{
    const bool rebuild = std::any_of(features.begin(), features.end(), needsStoppedStream);
    if (rebuild && streams_.size() != 1)
    {
        reason = "Live layout and trigger changes require a single configured stream";
        return false;
    }
    const bool acquiring = current_num_subscribers_ > 0;
    auto& stream = streams_.front();
    std::optional<Region> original_region;
    const auto original_format = stream.sensor.pixel_format;
    if (rebuild)
    {
        original_region = readRegion(p_camera_);
        if (!original_region)
        {
            reason = "Could not snapshot camera ROI";
            return false;
        }
        if (acquiring && !executeCommand("AcquisitionStop"))
        {
            reason = "Could not stop acquisition";
            return false;
        }
        if (!pauseStream(stream))
        {
            reason = "Frame publication is busy, retry the update";
            if (acquiring && !executeCommand("AcquisitionStart"))
            {
                reason += ". Could not resume acquisition";
                camera_unavailable_->store(true);
            }
            return false;
        }
    }

    const bool original_sampling_changed = output_sampling_changed_;
    std::vector<rclcpp::Parameter> undo;
    for (const auto& requested : features)
    {
        rclcpp::Parameter actual;
        const auto& name = requested.get_name();
        if (!readDynamicFeature(requested, actual))
        {
            reason = "Could not read " + name;
            break;
        }
        if (matches(actual, requested))
            continue;
        // Capture each bank after any preceding selector writes, including the attempted write.
        undo.push_back(actual);
        if (!writeDynamicFeature(requested) || !readDynamicFeature(requested, actual) ||
            !matches(actual, requested))
        {
            reason = "Camera rejected or adjusted " + name;
            break;
        }
        if (samplingFeature(name))
            output_sampling_changed_ = true;
    }
    if (reason.empty() && rebuild && !resumeStream(stream, acquiring))
        reason = "Could not resume acquisition with the new stream layout";
    if (reason.empty())
        return true;

    const auto restoreValue = [&](const rclcpp::Parameter& value)
    {
        rclcpp::Parameter current;
        if (readDynamicFeature(value, current) && matches(current, value))
            return true;
        return writeDynamicFeature(value) && readDynamicFeature(value, current) &&
          matches(current, value);
    };
    bool restored = true;
    for (auto it = undo.rbegin(); it != undo.rend(); ++it)
        restored &= restoreValue(*it);
    if (rebuild)
    {
        // Sampling and reversal writes can also change the pixel format or ROI implicitly.
        restored &= restoreValue(rclcpp::Parameter("PixelFormat", original_format));
        if (!cameraUnavailable() && readRegion(p_camera_) != original_region)
        {
            const auto& r = *original_region;
            GuardedGError err;
            arv_camera_set_region(p_camera_, r[0], r[1], r[2], r[3], err.ref());
            restored &= !err && readRegion(p_camera_) == original_region;
        }
        stream.sensor.pixel_format = original_format;
        output_sampling_changed_ = original_sampling_changed;
        if (!restored || !resumeStream(stream, acquiring))
        {
            // Aravis expects its receive thread to be running when the stream is finalized.
            arv_stream_start_thread(stream.p_arv_stream);
            restored = false;
        }
    }
    if (!restored)
    {
        executeCommand("AcquisitionStop");
        camera_unavailable_->store(true);
        reason += ". Rollback failed, reopen the camera";
        RCLCPP_ERROR(logger_, "%s", reason.c_str());
    }
    return false;
}

}  // namespace camera_aravis2
