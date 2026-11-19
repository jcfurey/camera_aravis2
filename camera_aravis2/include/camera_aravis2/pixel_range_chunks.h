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

#ifndef CAMERA_ARAVIS2__PIXEL_RANGE_CHUNKS_H_
#define CAMERA_ARAVIS2__PIXEL_RANGE_CHUNKS_H_

#include <cstdint>

#include <camera_aravis2_msgs/msg/frame_metadata.hpp>
#include "camera_aravis2/integer_chunk.h"

namespace camera_aravis2
{

class PixelRangeChunks
{
  public:
    explicit PixelRangeChunks(ArvGc* genicam) :
      min_(genicam, "ChunkPixelDynamicRangeMin"),
      max_(genicam, "ChunkPixelDynamicRangeMax")
    {
    }

    bool supported() const { return min_.supported() && max_.supported(); }

    void read(ArvChunkParser* parser, ArvBuffer* buffer,
              camera_aravis2_msgs::msg::FrameMetadata& msg) const
    {
        msg.pixel_dynamic_range_valid = false;
        msg.pixel_dynamic_range_min = 0;
        msg.pixel_dynamic_range_max = 0;
        if (!parser || !supported())
            return;

        const auto low = min_.read(parser, buffer);
        const auto high = max_.read(parser, buffer);
        if (low && high && *low >= 0 && *low < *high)
        {
            msg.pixel_dynamic_range_min = static_cast<uint64_t>(*low);
            msg.pixel_dynamic_range_max = static_cast<uint64_t>(*high);
            msg.pixel_dynamic_range_valid = true;
        }
    }

  private:
    IntegerChunk min_;
    IntegerChunk max_;
};

}  // namespace camera_aravis2

#endif  // CAMERA_ARAVIS2__PIXEL_RANGE_CHUNKS_H_
