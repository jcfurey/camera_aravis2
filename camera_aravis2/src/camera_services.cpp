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
#include <chrono>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "camera_aravis2/camera_driver.h"

namespace camera_aravis2
{

void CameraDriver::triggerImage(std_srvs::srv::Trigger::Response& response)
{
    std::string value;
    if (!isInitialized() || current_num_subscribers_ == 0)
        response.message = "A connected camera and a frame subscriber are required";
    else if (arv_device_get_feature(p_device_, "TriggerSelector") &&
             (!getFeatureValue("TriggerSelector", value) || value != "FrameStart"))
        response.message = "Select the FrameStart trigger";
    else if (!getFeatureValue("TriggerMode", value) || value != "On" ||
             !getFeatureValue("TriggerSource", value) || value != "Software")
        response.message = "Software triggering is disabled";
    else
    {
        bool armed = false;
        if (arv_device_is_feature_available(p_device_, "TriggerArmed", nullptr) &&
            (!getFeatureValue("TriggerArmed", armed) || !armed))
            response.message = "Camera trigger is not armed";
        else
        {
            response.success = executeCommand("TriggerSoftware");
            response.message = response.success ? "Trigger accepted" : "Trigger command failed";
        }
    }
}

void CameraDriver::describeFeature(const std::string& name,
                                   camera_aravis2_msgs::srv::GetFeature::Response& res)
{
    if (!isInitialized() || name.empty())
    {
        res.message = name.empty() ? "A feature name is required" : "Camera unavailable";
        return;
    }
    GuardedGError err;
    auto* feature = arv_device_get_feature(p_device_, name.c_str());
    if (!ARV_IS_GC_FEATURE_NODE(feature))
    {
        res.message = "Feature not implemented";
        res.success = true;
        return;
    }
    auto* node = ARV_GC_FEATURE_NODE(feature);
    res.available = arv_gc_feature_node_is_implemented(node, err.ref()) &&
      !err && arv_gc_feature_node_is_available(node, err.ref());
    if (err || !res.available)
    {
        res.success = !err;
        res.message = err ? err->message : "Feature unavailable in this configuration";
        return;
    }
    const auto access = arv_gc_feature_node_get_actual_access_mode(node);
    const bool locked = arv_gc_feature_node_is_locked(node, err.ref());
    if (err)
    {
        res.message = err->message;
        return;
    }
    const bool command = ARV_IS_GC_COMMAND(feature);
    const bool category = ARV_IS_GC_CATEGORY(feature);
    res.readable = !command && !category &&
      (access == ARV_GC_ACCESS_MODE_RO || access == ARV_GC_ACCESS_MODE_RW);
    res.writable = !category && !locked &&
      (access == ARV_GC_ACCESS_MODE_WO || access == ARV_GC_ACCESS_MODE_RW);
    if (res.readable)
    {
        const auto* value = arv_gc_feature_node_get_value_as_string(node, err.ref());
        if (err || !value)
        {
            res.message = err ? err->message : "No text representation";
            return;
        }
        res.value = value;
    }
    const auto bounds = [&](auto* number, auto get_min, auto get_max, auto get_inc, auto get_unit)
    {
        const auto field = [&](auto getter, std::string& target)
        {
            GuardedGError bound_error;
            const auto value = getter(number, bound_error.ref());
            if (!bound_error)
            {
                std::ostringstream text;
                text << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
                target = text.str();
            }
            else if (!g_error_matches(bound_error.operator->(), ARV_GC_ERROR,
                                      ARV_GC_ERROR_PROPERTY_NOT_DEFINED))
                res.message = bound_error->message;
        };
        field(get_min, res.minimum);
        field(get_max, res.maximum);
        field(get_inc, res.increment);
        const auto* unit = get_unit(number);
        if (unit)
            res.unit = unit;
    };
    if (ARV_IS_GC_ENUMERATION(feature))
    {
        res.type = "enum";
        guint count = 0;
        const char** entries = arv_gc_enumeration_dup_available_string_values(
          ARV_GC_ENUMERATION(feature), &count, err.ref());
        for (guint i = 0; entries && i < count; ++i)
            res.choices.emplace_back(entries[i]);
        g_free(entries);
    }
    else if (ARV_IS_GC_FLOAT(feature))
    {
        res.type = "float";
        bounds(ARV_GC_FLOAT(feature), arv_gc_float_get_min, arv_gc_float_get_max,
               arv_gc_float_get_inc, arv_gc_float_get_unit);
    }
    else if (ARV_IS_GC_INTEGER(feature))
    {
        res.type = "int";
        bounds(ARV_GC_INTEGER(feature), arv_gc_integer_get_min, arv_gc_integer_get_max,
               arv_gc_integer_get_inc, arv_gc_integer_get_unit);
    }
    else if (command)
        res.type = "command";
    else if (category)
        res.type = "category";
    else
        res.type = ARV_IS_GC_BOOLEAN(feature) ? "bool" : "string";
    if (err)
        res.message = err->message;
    res.success = res.message.empty();
}

bool CameraDriver::startAcquisition()
{
    // An uncertain acknowledgement at a slow frame rate can precede a valid image.
    // Observe Aravis's receiver directly, including when new-buffer signals are paused.
    const auto completed = [this]()
    {
        guint64 total = 0;
        for (const auto& stream : streams_)
            if (stream.p_arv_stream)
            {
                guint64 count = 0;
                arv_stream_get_statistics(stream.p_arv_stream, &count, nullptr, nullptr);
                total += count;
            }
        return total;
    };
    const auto before = completed();
    GuardedGError err;
    arv_device_execute_command(p_device_, "AcquisitionStart", err.ref());
    if (!err)
        return true;
    if (!g_error_matches(err.operator->(), ARV_DEVICE_ERROR, ARV_DEVICE_ERROR_TIMEOUT) ||
        streams_.empty() || streams_.front().frame_expectation != FrameExpectation::Continuous)
    {
        err.log(logger_, __FILE__, __LINE__, "AcquisitionStart");
        return false;
    }
    // Allow the same silence budget as the stream watchdog, with a bounded command wait.
    const auto grace = std::clamp(stall_timeout_, std::chrono::milliseconds(3000),
                                  std::chrono::milliseconds(30000));
    const auto deadline = std::chrono::steady_clock::now() + grace;
    while (rclcpp::ok(get_node_base_interface()->get_context()) && !cameraUnavailable() &&
           std::chrono::steady_clock::now() < deadline)
    {
        if (completed() > before)
        {
            RCLCPP_WARN(logger_, "AcquisitionStart timed out; a completed frame confirms capture.");
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    err.log(logger_, __FILE__, __LINE__, "AcquisitionStart produced no completed buffer");
    return false;
}

}  // namespace camera_aravis2
