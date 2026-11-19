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

#include "camera_aravis2/calibration_geometry.h"

#include <cmath>

namespace camera_aravis2
{
namespace
{

// Map one coordinate from the original pinhole image onto the published pixel grid.
bool axisTransform(const SensorAxis& axis, int offset, int length, double phase,
                   const CalibrationOptions& options, double& scale, double& translation)
{
    if (axis.pixels <= 0 || axis.step <= 0 || offset < 0 || length <= 0 ||
        !std::isfinite(phase) || phase < 0 || phase >= axis.step)
        return false;

    const int sampled_extent = axis.pixels / axis.step;
    const double crop = options.roi_in_sensor_pixels ?
      static_cast<double>(offset) / axis.step : offset;
    if (crop + length > sampled_extent)
        return false;

    scale = 1.0 / axis.step;
    translation = -phase * scale - crop;
    if (axis.reversed)
    {
        scale = -scale;
        translation = options.reverse_before_roi ?
          sampled_extent - 1 - crop - phase * scale : length - 1 - translation;
    }
    return true;
}

}  // namespace

sensor_msgs::msg::CameraInfo cameraInfoForFrame(sensor_msgs::msg::CameraInfo source,
  const CalibrationOptions& options, const CalibrationGeometry& geometry, const ImageRoi& roi,
  const std_msgs::msg::Header& header)
{
    sensor_msgs::msg::CameraInfo unknown;
    unknown.header = header;
    unknown.width = roi.width > 0 ? roi.width : 0;
    unknown.height = roi.height > 0 ? roi.height : 0;
    if (!geometry.known || !unknown.width || !unknown.height || source.k[0] == 0.0)
        return unknown;

    source.header = header;
    if (!options.full_sensor)
    {
        if (source.width != unknown.width || source.height != unknown.height ||
            roi.x != geometry.startup_roi.x || roi.y != geometry.startup_roi.y ||
            roi.width != geometry.startup_roi.width || roi.height != geometry.startup_roi.height)
            return unknown;
        return source;
    }

    // A sensor-space calibration must describe the original, unbinned grid.
    const bool whole_roi = (source.roi.width == 0 && source.roi.height == 0) ||
      (source.roi.width == source.width && source.roi.height == source.height);
    if (geometry.x.pixels <= 0 || geometry.y.pixels <= 0 ||
        source.width != static_cast<unsigned>(geometry.x.pixels) ||
        source.height != static_cast<unsigned>(geometry.y.pixels) ||
        source.binning_x > 1 || source.binning_y > 1 || source.roi.x_offset ||
        source.roi.y_offset || !whole_roi)
        return unknown;

    std::array<double, 2> scale{}, translation{};
    if (!axisTransform(geometry.x, roi.x, roi.width, options.sampling_offset[0], options,
                       scale[0], translation[0]) ||
        !axisTransform(geometry.y, roi.y, roi.height, options.sampling_offset[1], options,
                       scale[1], translation[1]))
        return unknown;

    // Left-multiply K and P by the sensor-to-image affine transform. This also
    // transforms skew and the stereo baseline term. D and R stay in optical coordinates.
    for (size_t row = 0; row < 2; ++row)
    {
        for (size_t col = 0; col < 3; ++col)
            source.k[3 * row + col] = scale[row] * source.k[3 * row + col] +
                                      translation[row] * source.k[6 + col];
        for (size_t col = 0; col < 4; ++col)
            source.p[4 * row + col] = scale[row] * source.p[4 * row + col] +
                                      translation[row] * source.p[8 + col];
    }
    source.width = unknown.width;
    source.height = unknown.height;
    source.binning_x = source.binning_y = 0;
    source.roi = sensor_msgs::msg::RegionOfInterest();
    return source;
}

}  // namespace camera_aravis2
