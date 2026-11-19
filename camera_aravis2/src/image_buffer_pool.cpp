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

#include "camera_aravis2/image_buffer_pool.h"

// Std
#include <algorithm>
#include <cstring>
#include <functional>
#include <limits>

namespace camera_aravis2
{

//==================================================================================================
ImageBufferPool::ImageBufferPool(const rclcpp::Logger& logger,
                                 ArvStream* stream,
                                 size_t payload_size_bytes,
                                 size_t n_preallocated_buffers) :
  stream_(stream),
  payload_size_bytes_(payload_size_bytes),
  n_buffers_(0),
  self_(this, [](ImageBufferPool*) {}),
  logger_(logger)
{
    allocateBuffers(n_preallocated_buffers);
}

//==================================================================================================
ImageBufferPool::~ImageBufferPool()
{
}

//==================================================================================================
sensor_msgs::msg::Image::SharedPtr ImageBufferPool::getRecyclableImg()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (dangling_imgs_.empty())
    {
        return sensor_msgs::msg::Image::SharedPtr(new sensor_msgs::msg::Image,
                                                  std::bind(&ImageBufferPool::reclaim,
                                                            this->weak_from_this(),
                                                            std::placeholders::_1));
    }
    else
    {
        sensor_msgs::msg::Image::SharedPtr img_ptr = dangling_imgs_.top();
        dangling_imgs_.pop();
        return img_ptr;
    }
}

//==================================================================================================
sensor_msgs::msg::Image::SharedPtr ImageBufferPool::imagePixels(
  ArvBuffer* buffer, const sensor_msgs::msg::Image::SharedPtr& received)
{
    size_t available = 0;
    const auto* base = static_cast<const uint8_t*>(arv_buffer_get_data(buffer, &available));
    const auto* pixels = base;
    gint x_padding = 0;
#if ARAVIS_MAJOR_VERSION > 0 || ARAVIS_MINOR_VERSION > 8 || ARAVIS_MICRO_VERSION >= 23
    if (arv_buffer_get_n_parts(buffer) == 0)
        return nullptr;
    const auto type = arv_buffer_get_part_data_type(buffer, 0);
    if (type < ARV_BUFFER_PART_DATA_TYPE_2D_IMAGE ||
        type > ARV_BUFFER_PART_DATA_TYPE_CONFIDENCE_MAP)
        return nullptr;
    size_t part_size = 0;
    pixels = static_cast<const uint8_t*>(arv_buffer_get_part_data(buffer, 0, &part_size));
    if (!base || !pixels || pixels < base || static_cast<size_t>(pixels - base) > available)
        return nullptr;
    available = std::min(part_size, available - static_cast<size_t>(pixels - base));
    arv_buffer_get_image_padding(buffer, &x_padding, nullptr);
#else
    const auto type = arv_buffer_get_payload_type(buffer);
    if (type != ARV_BUFFER_PAYLOAD_TYPE_IMAGE &&
        type != ARV_BUFFER_PAYLOAD_TYPE_EXTENDED_CHUNK_DATA)
        return nullptr;
#endif
    const gint width = arv_buffer_get_image_width(buffer);
    const gint height = arv_buffer_get_image_height(buffer);
    const auto format = arv_buffer_get_image_pixel_format(buffer);
    const auto bits = ARV_PIXEL_FORMAT_BIT_PER_PIXEL(format);
    if (!pixels || width <= 0 || height <= 0 || bits == 0 || x_padding < 0)
        return nullptr;
    // The converters consume whole packing groups in tightly packed, byte-aligned rows.
    const uint64_t row_bits = static_cast<uint64_t>(width) * bits;
    const auto row_bytes = row_bits / 8;
    if (row_bits % 8 != 0 || row_bytes > std::numeric_limits<uint32_t>::max())
        return nullptr;
    // In planar RGB each component has its own scanlines and end-of-line padding.
    const bool planar = format == ARV_PIXEL_FORMAT_RGB_8_PLANAR ||
      format == ARV_PIXEL_FORMAT_RGB_10_PLANAR || format == ARV_PIXEL_FORMAT_RGB_12_PLANAR ||
      format == ARV_PIXEL_FORMAT_RGB_16_PLANAR;
    const auto planes = planar ? 3u : 1u;
    const auto line_bytes = row_bytes / planes;
    const auto lines = static_cast<uint64_t>(height) * planes;
    const auto stride = line_bytes + x_padding;
    if (stride > available / lines)
        return nullptr;

    const size_t size = static_cast<size_t>(row_bytes * height);
    auto img = received;
    if (pixels != received->data.data() || size != received->data.size() || x_padding != 0)
    {
        // Aravis still owns the receive vector's storage. Resize only an unregistered image.
        img = getRecyclableImg();
        img->data.resize(size);
        if (x_padding == 0)
            std::memcpy(img->data.data(), pixels, size);
        else
            for (size_t row = 0; row < lines; ++row)
                std::memcpy(img->data.data() + row * line_bytes, pixels + row * stride, line_bytes);
        img->is_bigendian = received->is_bigendian;
    }
    img->step = static_cast<uint32_t>(row_bytes);
    return img;
}

//==================================================================================================
sensor_msgs::msg::Image::SharedPtr ImageBufferPool::operator[](ArvBuffer* buffer)
{
    std::lock_guard<std::mutex> lock(mutex_);
    sensor_msgs::msg::Image::SharedPtr img_ptr;
    if (buffer)
    {
        // get address and size
        size_t buffer_size;
        const uint8_t* buffer_data = (const uint8_t*)arv_buffer_get_data(buffer, &buffer_size);

        // find corresponding ImagePtr wrapper
        std::map<const uint8_t*, sensor_msgs::msg::Image::SharedPtr>::iterator iter =
          available_img_buffers_.find(buffer_data);
        if (iter != available_img_buffers_.end())
        {
            img_ptr = iter->second;
            used_buffers_.emplace(img_ptr.get(), buffer);
            available_img_buffers_.erase(iter);
        }
        else
        {
            RCLCPP_WARN(logger_, "Could not find available image in pool corresponding to buffer.");
            img_ptr.reset(new sensor_msgs::msg::Image);
            img_ptr->data.resize(buffer_size);
            memcpy(img_ptr->data.data(), buffer_data, buffer_size);
        }
    }

    return img_ptr;
}

//==================================================================================================
sensor_msgs::msg::Image::SharedPtr ImageBufferPool::acquireFrame(ArvBuffer* buffer,
                                                                 size_t max_buffers)
{
    // Only the stream's receive callback allocates after startup.
    gint remaining;
    arv_stream_get_n_buffers(stream_, &remaining, nullptr);
    if (remaining <= 0)
    {
        if (getAllocatedSize() >= max_buffers)
        {
            arv_stream_push_buffer(stream_, buffer);
            return nullptr;
        }
        allocateBuffers(1);
    }
    return (*this)[buffer];
}

//==================================================================================================
void ImageBufferPool::allocateBuffers(size_t n)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (ARV_IS_STREAM(stream_))
    {
        for (size_t i = 0; i < n; ++i)
        {
            sensor_msgs::msg::Image* p_img = new sensor_msgs::msg::Image();
            p_img->data.resize(payload_size_bytes_);
            ArvBuffer* buffer = arv_buffer_new(payload_size_bytes_, p_img->data.data());
            sensor_msgs::msg::Image::SharedPtr img_ptr(p_img,
                                                       std::bind(&ImageBufferPool::reclaim,
                                                                 this->weak_from_this(),
                                                                 std::placeholders::_1));
            available_img_buffers_.emplace(p_img->data.data(), img_ptr);
            arv_stream_push_buffer(stream_, buffer);
            ++n_buffers_;
        }
        RCLCPP_INFO_STREAM(logger_,
                           "Allocated " << n << " image buffers of size " << payload_size_bytes_);
    }
    else
    {
        RCLCPP_ERROR(logger_, "Error: Stream not valid. Failed to allocate buffers.");
    }
}

//==================================================================================================
void ImageBufferPool::reclaim(const WeakPtr& self, sensor_msgs::msg::Image* p_img)
{
    SharedPtr s = self.lock();
    if (s)
    {
        s->push(p_img);
    }
    else
    {
        delete p_img;
    }
}

//==================================================================================================
void ImageBufferPool::push(sensor_msgs::msg::Image* p_img)
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::map<sensor_msgs::msg::Image*, ArvBuffer*>::iterator iter = used_buffers_.find(p_img);

    if (iter != used_buffers_.end())
    {
        if (ARV_IS_STREAM(stream_))
        {
            sensor_msgs::msg::Image::SharedPtr img_ptr(p_img,
                                                       std::bind(&ImageBufferPool::reclaim,
                                                                 this->weak_from_this(),
                                                                 std::placeholders::_1));
            available_img_buffers_.emplace(p_img->data.data(), img_ptr);
            arv_stream_push_buffer(stream_, iter->second);
        }
        else
        {
            // the camera stream is gone, so should its buffers
            delete p_img;
        }
        used_buffers_.erase(iter);
    }
    else
    {
        // this image was not an aravis registered buffer
        dangling_imgs_.emplace(p_img,
                               std::bind(&ImageBufferPool::reclaim,
                                         this->weak_from_this(),
                                         std::placeholders::_1));
    }
}

}  // namespace camera_aravis2
