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
#include <functional>
#include <set>
#include <string>
#include <vector>

#include "camera_aravis2/camera_driver.h"

namespace camera_aravis2
{

bool CameraDriver::captureSelectors(SavedFeature& setting) const
{
    // Hidden GenICam selectors can address a feature just as visible ones can.
    std::vector<ArvGcFeatureNode*> selectors;
    const std::function<void(ArvDomNode*)> visit = [&](ArvDomNode* node)
    {
        if (ARV_IS_GC_SELECTOR(node) && arv_gc_selector_is_selector(ARV_GC_SELECTOR(node)))
            selectors.push_back(ARV_GC_FEATURE_NODE(node));
        for (auto* child = arv_dom_node_get_first_child(node); child;
             child = arv_dom_node_get_next_sibling(child))
            visit(child);
    };
    visit(ARV_DOM_NODE(arv_device_get_genicam(p_device_)));
    std::set<ArvGcFeatureNode*> visited, active;
    const std::function<bool(ArvGcNode*)> capture = [&](ArvGcNode* target)
    {
        for (auto* selector : selectors)
        {
            const auto* features = arv_gc_selector_get_selected_features(ARV_GC_SELECTOR(selector));
            if (visited.count(selector) || !g_slist_find(const_cast<GSList*>(features), target))
                continue;
            if (!active.insert(selector).second || !capture(ARV_GC_NODE(selector)))
                return false;
            GuardedGError err;
            const char* value = arv_gc_feature_node_get_value_as_string(selector, err.ref());
            if (err || !value)
                return false;
            setting.selectors.emplace_back(arv_gc_feature_node_get_name(selector), value);
            visited.insert(selector);
            active.erase(selector);
        }
        return true;
    };
    const auto& name = setting.value.get_name();
    if (!capture(arv_device_get_feature(p_device_, name.c_str())))
        return false;
    // Some GenICam descriptions implement standard banks with indexed registers but omit
    // pSelected. Their public selector still has to be restored for each accepted write.
    const char* bank = nullptr;
    if (name == "TriggerMode" || name == "TriggerSource" || name == "TriggerActivation" ||
        name == "TriggerDelay")
        bank = "TriggerSelector";
    else if (name == "ChunkEnable")
        bank = "ChunkSelector";
    else if (name == "Gain" || name == "GainRaw")
        bank = "GainSelector";
    else if (name == "BlackLevel" || name == "BlackLevelRaw")
        bank = "BlackLevelSelector";
    else if (name == "BalanceRatio" || name == "BalanceRatioRaw")
        bank = "BalanceRatioSelector";
    if (bank)
    {
        auto* node = arv_device_get_feature(p_device_, bank);
        if (ARV_IS_GC_FEATURE_NODE(node) && !visited.count(ARV_GC_FEATURE_NODE(node)))
        {
            std::string value;
            if (!capture(node) || !getFeatureValue(bank, value))
                return false;
            setting.selectors.emplace_back(bank, value);
        }
    }
    return true;
}

bool CameraDriver::replayFeatures()
{
    using Bank = std::pair<std::string, std::string>;
    const auto select = [this](const Bank& bank, std::vector<Bank>* restore)
    {
        auto* feature = arv_device_get_feature(p_device_, bank.first.c_str());
        if (!ARV_IS_GC_FEATURE_NODE(feature))
            return false;
        auto* node = ARV_GC_FEATURE_NODE(feature);
        GuardedGError err;
        const char* current = arv_gc_feature_node_get_value_as_string(node, err.ref());
        if (err || !current)
            return false;
        if (current == bank.second)
            return true;
        if (restore)
            restore->emplace_back(bank.first, current);
        arv_gc_feature_node_set_value_from_string(node, bank.second.c_str(), err.ref());
        return !err;
    };
    for (const auto& entry : saved_features_)
    {
        const auto& name = entry.value.get_name();
        if (saved_region_ && (name == "Width" || name == "Height" ||
                              name == "OffsetX" || name == "OffsetY"))
            continue;
        std::vector<Bank> restore;
        bool ok = std::all_of(entry.selectors.begin(), entry.selectors.end(),
                              [&](const Bank& bank) { return select(bank, &restore); });
        rclcpp::Parameter current;
        if (ok && (!readDynamicFeature(entry.value, current) || current != entry.value))
            ok = writeDynamicFeature(entry.value) && readDynamicFeature(entry.value, current) &&
              current == entry.value;
        for (auto i = restore.rbegin(); i != restore.rend(); ++i)
            ok &= select(*i, nullptr);
        if (!ok)
        {
            RCLCPP_ERROR(logger_, "Could not restore %s on reconnect", name.c_str());
            return false;
        }
    }
    if (saved_region_)
    {
        const auto& r = *saved_region_;
        GuardedGError err;
        arv_camera_set_region(p_camera_, r[0], r[1], r[2], r[3], err.ref());
        if (err)
            return false;
        std::array<int, 4> actual{};
        arv_camera_get_region(p_camera_, &actual[0], &actual[1], &actual[2], &actual[3], err.ref());
        return !err && actual == r;
    }
    return true;
}

bool CameraDriver::reopenCamera()
{
    // Late signals from the previous device retain only its own unavailable flag.
    std::atomic_store(&camera_unavailable_, std::make_shared<std::atomic<bool>>(false));
    if (!discoverAndOpenCameraDevice(1, hardware_id_) ||
        constructCameraGuidStr(p_camera_) != hardware_id_ ||
        (arv_camera_is_gv_device(p_camera_) &&
         !arv_gv_device_is_controller(ARV_GV_DEVICE(p_device_))))
        return false;
    if (discoverNumberOfStreams() < static_cast<int>(streams_.size()))
        return false;

    config_warn_msgs_.clear();
    for (auto& stream : streams_)
    {
        stream.analog_control = {};
        stream.acquisition_control = {};
        stream.frame_timestamp = {};
        stream.cvt_pixel_format = {};
        stream.previous_counters = {};
    }
    if (!setDeviceControlSettings() ||
        (arv_camera_is_gv_device(p_camera_) && !setTransportLayerControlSettings()) ||
        !setImageFormatControlSettings() || !setAcquisitionControlSettings() ||
        !setAnalogControlSettings() || !replayFeatures())
        return false;
    for (size_t i = 0; i < streams_.size(); ++i)
    {
        auto& stream = streams_[i];
        if (streams_.size() > 1 &&
            !setFeatureValue<std::string>("SourceSelector", "Source" + std::to_string(i)))
            return false;
        GuardedGError err;
        stream.pixel_format_code = arv_camera_get_pixel_format(p_camera_, err.ref());
        if (err || !getFeatureValue("PixelFormat", stream.sensor.pixel_format))
            return false;
        stream.sensor.n_bits_pixel = ARV_PIXEL_FORMAT_BIT_PER_PIXEL(stream.pixel_format_code);
        const auto converter = CONVERSIONS_DICTIONARY.find(stream.sensor.pixel_format);
        stream.cvt_pixel_format = converter == CONVERSIONS_DICTIONARY.end() ?
          ConversionFunction{} : converter->second;
    }
    return configureTimestampSource();
}

void CameraDriver::checkRecovery()
{
    if (!session_started_ || is_spawning_)
        return;
    {
        std::lock_guard<std::mutex> lock(acquisition_mutex_);
        if (!cameraUnavailable() && is_initialized_)
        {
            if (!recovery_requested_ || current_num_subscribers_ == 0)
                return;
            const auto stalled = [&](const Stream& stream)
            {
                return stream.p_arv_stream &&
                  stream.frame_expectation == FrameExpectation::Continuous &&
                  std::chrono::steady_clock::now() - stream.last_frame_activity >= stall_timeout_ &&
                  readStreamCounters(stream).completed == stream.previous_counters.completed;
            };
            if (std::none_of(streams_.begin(), streams_.end(), stalled))
                return;
        }
        is_initialized_ = false;
        recovery_requested_ = false;
    }
    RCLCPP_WARN(logger_, "Reopening camera %s", hardware_id_.c_str());
    stopCameraStreams();
    releaseCamera();
    current_num_subscribers_ = 0;
    if (reopenCamera())
    {
        {
            std::lock_guard<std::mutex> lock(worker_mutex_);
            stop_workers_ = false;
        }
        if (p_diagnostic_pub_)
        {
            const auto rate = get_parameter("diagnostic_publishing_rate").as_double();
            diagnostic_thread_ =
              std::thread(&CameraDriver::publishCameraDiagnosticsLoop, this, rate);
        }
        startCameraWorkers();
    }
    else
    {
        releaseCamera();
        camera_unavailable_->store(true);
    }
    stream_diagnostic_timer_->reset();
}

}  // namespace camera_aravis2
