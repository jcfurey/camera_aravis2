// Copyright (c) 2024 Fraunhofer IOSB and contributors
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

#include "camera_aravis2/camera_driver_gv.h"

// Std
#include <chrono>
#include <iostream>
#include <limits>
#include <thread>

// ROS
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rclcpp/time.hpp>

// camera_aravis2
#include "camera_aravis2/conversion_utils.h"
#include "camera_aravis2/error.h"

namespace camera_aravis2
{

//==================================================================================================
CameraDriverGv::CameraDriverGv(const rclcpp::NodeOptions& options) :
  CameraDriver("camera_driver_gv", options)
{
    //--- setup parameters
    setupParameters();

    rcl_interfaces::msg::ParameterDescriptor buffer_desc;
    buffer_desc.read_only = true;
    buffer_desc.description = "GigE stream UDP receive buffer in bytes. Zero uses the OS default.";
    rcl_interfaces::msg::IntegerRange buffer_range;
    buffer_range.from_value = 0;
    buffer_range.to_value = std::numeric_limits<gint>::max();
    buffer_range.step = 1;
    buffer_desc.integer_range.push_back(buffer_range);
    declare_parameter<int64_t>("stream_socket_buffer_size", 0, buffer_desc);

    rcl_interfaces::msg::ParameterDescriptor ptp_desc;
    ptp_desc.read_only = true;
    ptp_desc.description = "Maximum absolute slave offset for PTP lock in nanoseconds.";
    const auto tolerance = declare_parameter<int64_t>("ptp_max_offset_ns", 100000, ptp_desc);
    ptp_desc.description = "Seconds of continuously good PTP observations required for lock.";
    const auto window = declare_parameter<int64_t>("ptp_offset_window_s", 5, ptp_desc);
    ptp_desc.description = "Maximum skew in seconds between corrected PTP and host reception.";
    const auto skew_s = declare_parameter<double>("ptp_max_host_skew_s", 1.0, ptp_desc);
    const auto skew = timestampOffsetNs(skew_s);
    if (tolerance < 0 || window < 0 || !skew || skew_s < 0)
    {
        RCLCPP_ERROR(logger_, "Invalid PTP tolerance, lock window or host skew");
        return;
    }
    ptp_max_host_skew_ns_ = static_cast<uint64_t>(*skew);

    is_verbose_enable_ = get_parameter("verbose").as_bool();

    //--- get parameter overrides, i.e. all parameters, including those that are not declared
    parameter_overrides_ = this->get_node_parameters_interface()->get_parameter_overrides();

    //--- open camera device
    CHECK_SUCCESS(discoverAndOpenCameraDevice(), logger_);

    //--- check if GEV Device.
    if (!arv_camera_is_gv_device(p_camera_))
    {
        RCLCPP_FATAL(logger_, "Camera is no GigE-Vision Device.");
        return;
    }

    //--- set up structs holding relevant information of camera streams
    CHECK_SUCCESS(setupCameraStreamStructs(), logger_);

    //--- set device control settings
    CHECK_SUCCESS(setDeviceControlSettings(), logger_);

    //--- set transport layer control settings
    p_gv_tl_control_ = std::make_shared<GvTransportLayerControl>();
    p_tl_control_    = std::dynamic_pointer_cast<GenTransportLayerControl>(p_gv_tl_control_);

    CHECK_SUCCESS(setTransportLayerControlSettings(), logger_);
    timestamp_source_ = timestampSource(get_parameter("timestamp_mode").as_string(),
                                        p_gv_tl_control_->is_ptp_enable);

    if (timestamp_source_ == TimestampSource::Ptp && !p_gv_tl_control_->is_ptp_enable)
    {
        RCLCPP_ERROR(logger_, "timestamp_mode=ptp requires PtpEnable to be true on the camera");
        return;
    }

    //--- set image format control settings
    CHECK_SUCCESS(setImageFormatControlSettings(), logger_);

    //--- set acquisition control settings
    CHECK_SUCCESS(setAcquisitionControlSettings(), logger_);

    //--- set analog control settings
    CHECK_SUCCESS(setAnalogControlSettings(), logger_);

    //--- initialize services
    CHECK_SUCCESS(initializeServices(), logger_);

    //--- load dynamic parameters
    setupDynamicParameters();

    //--- load diagnostics
    setupCameraDiagnosticPublisher();

    //--- check ptp
    if (p_gv_tl_control_->is_ptp_enable)
    {
        ptp_lock_.emplace(std::chrono::seconds(window), tolerance);
        checkPtpState();
        p_gv_tl_control_->ptp_status = ptp_lock_->sample().status;
    }

    //--- print currently applied camera configuration
    printCameraConfiguration();

    //--- spawn camera stream in thread, so that initialization is not blocked
    is_spawning_         = true;
    spawn_stream_thread_ = std::thread([this]()
    {
        spawnCameraStreams();
        if (!isInitialized() || !ptp_lock_)
            return;

        // Keep polling on the startup thread so slow camera reads do not block the
        // executor or frame processing. stopCameraStreams wakes and joins this thread.
        std::unique_lock<std::mutex> lock(ptp_mutex_);
        while (!ptp_wakeup_.wait_for(lock, std::chrono::seconds(1),
                                     [this]() { return stop_ptp_polling_; }))
        {
            lock.unlock();
            checkPtpState();
            lock.lock();
        }
    });
}

//==================================================================================================
CameraDriverGv::~CameraDriverGv()
{
    stopCameraStreams();
}

//==================================================================================================
[[nodiscard]] bool CameraDriverGv::setTechSpecificTlControlSettings()
{
    GuardedGError err;

    std::string tmp_feature_name;
    rclcpp::ParameterValue tmp_param_value;
    std::vector<std::pair<std::string, rclcpp::ParameterValue>> tmp_param_values;

    bool is_parameter_set;

    //--- check if transport layer control is correctly initialized
    if (!p_gv_tl_control_)
    {
        RCLCPP_FATAL(logger_, "%s:  Pointer to GevTransportLayerControl is not initialized. ",
                     __PRETTY_FUNCTION__);

        return false;
    }

    //--- for specific parameters, proceed as follows:
    //---   1. Check and read parameter from launch parameters
    //---   2. If provided in launch parameter, set value on device
    //---   3. Get the set value from the device and store it in corresponding struct
    //---   4. If specific parameter was given in the launch parameters, check against the actual
    //---       value which has been read from the camera and issue a warning message if applicable

    //--- GevSCPSPacketSize
    tmp_feature_name = "GevSCPSPacketSize";
    RCLCPP_DEBUG(logger_, "Evaluating 'TransportLayerControl.%s'", tmp_feature_name.c_str());
    is_parameter_set = getTransportLayerControlParameter(tmp_feature_name, tmp_param_value);
    if (is_parameter_set)
        setFeatureValueFromParameter<int64_t>(tmp_feature_name, tmp_param_value);
    getFeatureValue<int64_t>(tmp_feature_name, p_gv_tl_control_->packet_size);
    if (is_parameter_set &&
        !isParameterValueEqualTo<int64_t>(tmp_param_value, p_gv_tl_control_->packet_size))
        config_warn_msgs_.push_back("'" + tmp_feature_name + "' is not as specified.");

    //--- GevSCPD
    tmp_feature_name = "GevSCPD";
    RCLCPP_DEBUG(logger_, "Evaluating 'TransportLayerControl.%s'", tmp_feature_name.c_str());
    is_parameter_set = getTransportLayerControlParameter(tmp_feature_name, tmp_param_value);
    if (is_parameter_set)
        setFeatureValueFromParameter<int64_t>(tmp_feature_name, tmp_param_value);
    getFeatureValue<int64_t>(tmp_feature_name, p_gv_tl_control_->inter_packet_delay);
    if (is_parameter_set &&
        !isParameterValueEqualTo<int64_t>(tmp_param_value, p_gv_tl_control_->inter_packet_delay))
        config_warn_msgs_.push_back("'" + tmp_feature_name + "' is not as specified.");

    //--- set PTP Enable
    tmp_feature_name = "PtpEnable";
    RCLCPP_DEBUG(logger_, "Evaluating 'TransportLayerControl.%s'", tmp_feature_name.c_str());
    is_parameter_set = getTransportLayerControlParameter(tmp_feature_name, tmp_param_value);
    if (is_parameter_set)
        setFeatureValueFromParameter<bool>(tmp_feature_name, tmp_param_value);
    getFeatureValue<bool>(tmp_feature_name, p_gv_tl_control_->is_ptp_enable);
    if (is_parameter_set &&
        !isParameterValueEqualTo<bool>(tmp_param_value, p_gv_tl_control_->is_ptp_enable))
        config_warn_msgs_.push_back("'" + tmp_feature_name + "' is not as specified.");

    return true;
}

//==================================================================================================
int CameraDriverGv::discoverNumberOfStreams()
{
    //--- upper bound for a plausible number of stream channels (see #62)
    constexpr int64_t MAX_NUM_STREAMS = 16;

    int64_t num_streams = 0;

    if (p_device_)
    {
        num_streams = arv_device_get_integer_feature_value(p_device_, "DeviceStreamChannelCount",
                                                           nullptr);

        if (num_streams == 0 && arv_camera_is_gv_device(p_camera_))
        {
            num_streams = arv_device_get_integer_feature_value(p_device_, "GevStreamChannelCount",
                                                               nullptr);
        }
    }

    if (num_streams == 0 || !p_device_)
    {
        num_streams = 1;
        RCLCPP_INFO(logger_, "Unable to automatically detect number of supported stream channels. "
                             "Setting num_streams = %i.",
                    static_cast<int>(num_streams));
    }
    else if (num_streams < 0 || num_streams > MAX_NUM_STREAMS)
    {
        RCLCPP_WARN(logger_, "Invalid number of stream channels reported by device: %s. "
                             "Setting num_streams = 1.",
                    std::to_string(num_streams).c_str());
        num_streams = 1;
    }
    else
    {
        RCLCPP_INFO(logger_, "Number of supported stream channels: %i",
                    static_cast<int>(num_streams));
    }

    return static_cast<int>(num_streams);
}

//==================================================================================================
void CameraDriverGv::tuneArvStream(ArvStream* p_stream) const
{
    if (!p_stream)
        return;

    if (!ARV_IS_GV_STREAM(p_stream))
    {
        RCLCPP_ERROR(logger_, "Stream is not a GV_STREAM");
        return;
    }

    const auto buffer_size = get_parameter("stream_socket_buffer_size").as_int();
    gboolean b_packet_resend             = TRUE;
    unsigned int timeout_packet          = 40;  // milliseconds
    unsigned int timeout_frame_retention = 200;

    if (buffer_size > 0)
        g_object_set(p_stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_FIXED,
                     "socket-buffer-size", static_cast<gint>(buffer_size), nullptr);
    if (!b_packet_resend)
        g_object_set(p_stream, "packet-resend",
                     b_packet_resend
                       ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS
                       : ARV_GV_STREAM_PACKET_RESEND_NEVER,
                     NULL);
    g_object_set(p_stream, "packet-timeout",
                 timeout_packet * 1000,
                 "frame-retention", timeout_frame_retention * 1000,
                 NULL);
}

//==================================================================================================
void CameraDriverGv::postFrameProcessingCallback(const uint stream_id)
{
    RCL_UNUSED(stream_id);
}

//==================================================================================================
void CameraDriverGv::checkPtpState()
{
    const auto start = PtpLock::Clock::now();
    PtpSample sample;
    const bool latched = !arv_device_get_feature(p_device_, "PtpDataSetLatch") ||
      executeCommand("PtpDataSetLatch");
    std::string status;
    if (latched && getFeatureValue("PtpStatus", status))
        sample.status = status;
    if (sample.status == "Slave")
    {
        int64_t offset = 0;
        if (getFeatureValue("PtpOffsetFromMaster", offset))
        {
            // LUCID uses -1 for an unavailable offset. Other cameras can report -1 ns
            // as a valid signed offset.
            if (offset != -1)
                sample.offset_ns = offset;
            else
            {
                const char* vendor = arv_camera_get_vendor_name(p_camera_, nullptr);
                if (vendor && g_ascii_strncasecmp(vendor, "Lucid", 5) != 0)
                    sample.offset_ns = offset;
            }
        }
        if (arv_device_get_feature(p_device_, "PtpServoStatus"))
        {
            std::string servo;
            sample.servo_status = getFeatureValue("PtpServoStatus", servo) ? servo : "Unknown";
        }
    }
    const auto wall = std::chrono::system_clock::now().time_since_epoch();
    const auto host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall).count();
    std::lock_guard<std::mutex> lock(ptp_mutex_);
    ptp_lock_->observe(std::move(sample), start, host_ns);
}

}  // end namespace camera_aravis2

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_aravis2::CameraDriverGv)
