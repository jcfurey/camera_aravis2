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

#include <charconv>
#include <cmath>
#include <limits>
#include <string>

#include "camera_aravis2/camera_driver.h"

namespace camera_aravis2
{
namespace
{
unsigned decimal(const std::string& text, size_t pos, bool require_end)
{
    unsigned value = 0;
    if (pos >= text.size())
        return 0;
    const auto* end = text.data() + text.size();
    const auto result = std::from_chars(text.data() + pos, end, value);
    return result.ec == std::errc{} && (!require_end || result.ptr == end) ? value : 0;
}
}  // namespace

camera_aravis2_msgs::msg::FrameMetadata CameraDriver::measurementSnapshot(
  const Stream& stream) const
{
    camera_aravis2_msgs::msg::FrameMetadata msg;
    const auto& format = stream.sensor.pixel_format;
    msg.pixel_format_name = format;
    msg.intensity_range_source = "unavailable";
    // PFNC occupied bits include packing/padding and may cover several channels. For the
    // supported unsigned intensity formats the name encodes the significant bits per channel.
    if (stream.cvt_pixel_format)
    {
        for (const char* prefix : {"Mono", "Bayer", "RGB", "BGR", "Raw", "R", "G", "B", "YUV"})
        {
            if (format.rfind(prefix, 0) != 0)
                continue;
            const auto n = format.rfind("YUV", 0) == 0 ? 8 :
              decimal(format, format.find_first_of("0123456789"), false);
            if (n == 8 || n == 10 || n == 12 || n == 14 || n == 16)
            {
                msg.pixel_bit_depth = n;
                msg.sample_shift = n > 8 ? 16 - n : 0;
                msg.intensity_range_source = "pixel_format";
                msg.intensity_max = (uint64_t{1} << n) - 1;
            }
            break;
        }
    }
    int64_t low = 0, high = 0;
    if (msg.pixel_bit_depth && getFeatureValue("PixelDynamicRangeMin", low) &&
        getFeatureValue("PixelDynamicRangeMax", high) && low >= 0 && low < high &&
        static_cast<uint64_t>(high) <= msg.intensity_max)
    {
        msg.intensity_min = low;
        msg.intensity_max = high;
        msg.intensity_range_source = "camera";
    }
    std::string adc;
    if (getFeatureValue("ADCBitDepth", adc) && adc.rfind("Bits", 0) == 0)
    {
        const auto bits = decimal(adc, 4, true);
        msg.adc_bit_depth_valid = bits > 0 && bits <= 64;
        if (msg.adc_bit_depth_valid)
            msg.adc_bit_depth = bits;
    }
    auto* black = arv_device_get_feature(p_device_, "BlackLevel");
    const char* unit = nullptr;
    if (ARV_IS_GC_FLOAT(black))
    {
        msg.black_level_valid = getFeatureValue("BlackLevel", msg.black_level);
        unit = arv_gc_float_get_unit(ARV_GC_FLOAT(black));
    }
    else if (ARV_IS_GC_INTEGER(black))
    {
        int64_t value = 0;
        msg.black_level_valid = getFeatureValue("BlackLevel", value);
        msg.black_level = value;
        unit = arv_gc_integer_get_unit(ARV_GC_INTEGER(black));
    }
    if (unit)
        msg.black_level_unit = unit;
    if (arv_device_get_feature(p_device_, "BlackLevelSelector"))
        msg.black_level_valid &= getFeatureValue("BlackLevelSelector", msg.black_level_selector);
    if (!msg.black_level_valid)
        msg.black_level = std::numeric_limits<double>::quiet_NaN();

    const auto optional = [this](const char* name, auto& value)
    {
        return !arv_device_get_feature(p_device_, name) || getFeatureValue(name, value);
    };
    const auto sampling = [&](const char* selector, const char* horizontal, const char* vertical,
                              uint32_t& x, uint32_t& y)
    {
        x = y = 1;
        const auto accumulate = [&](const char* name, uint32_t& total)
        {
            int64_t factor = 1;
            if (!optional(name, factor) || factor < 1 || factor > UINT32_MAX / total)
                return false;
            total *= factor;
            return true;
        };
        if (!arv_device_get_feature(p_device_, selector))
            return accumulate(horizontal, x) && accumulate(vertical, y);
        std::string original;
        if (!getFeatureValue(selector, original))
            return false;
        guint count = 0;
        auto** banks = arv_camera_dup_available_enumerations_as_strings(p_camera_, selector,
                                                                       &count, nullptr);
        bool ok = banks && count;
        std::string current = original;
        for (guint i = 0; ok && i < count; ++i)
        {
            if (current != banks[i])
            {
                current = banks[i];
                ok = setFeatureValue(selector, current);
            }
            ok = ok && accumulate(horizontal, x) && accumulate(vertical, y);
        }
        g_free(banks);
        return (current == original || setFeatureValue(selector, original)) && ok;
    };
    bool rows = false, columns = false;
    int64_t imagers = 1;
    msg.geometry_valid = optional("MultipleROIRowsEnable", rows) && !rows &&
      optional("MultipleROIColumnsEnable", columns) && !columns &&
      optional("ImagerCount", imagers) && imagers == 1 &&
      sampling("BinningSelector", "BinningHorizontal", "BinningVertical", msg.binning_x,
               msg.binning_y) &&
      sampling("DecimationSelector", "DecimationHorizontal", "DecimationVertical", msg.decimation_x,
               msg.decimation_y) && optional("ReverseX", msg.reverse_x) &&
      optional("ReverseY", msg.reverse_y);
    return msg;
}

}  // namespace camera_aravis2
