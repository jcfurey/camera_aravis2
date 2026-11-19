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

#ifndef CAMERA_ARAVIS2__FRAME_IO_CHUNKS_H_
#define CAMERA_ARAVIS2__FRAME_IO_CHUNKS_H_

#include <arv.h>

#include <cstdint>
#include <optional>

#include <camera_aravis2_msgs/msg/frame_metadata.hpp>
#include "camera_aravis2/error.h"

namespace camera_aravis2
{

class FrameIoChunks
{
  public:
    explicit FrameIoChunks(ArvGc* genicam) :
      sequence_(integerFeature(genicam, "ChunkSequencerSetActive")),
      lines_(integerFeature(genicam, "ChunkLineStatusAll"))
    {
    }

    bool supported() const { return sequence_ || lines_; }

    void read(ArvChunkParser* parser, ArvBuffer* buffer,
              camera_aravis2_msgs::msg::FrameMetadata& msg) const
    {
        const auto sequence = readInteger(parser, buffer, sequence_);
        msg.sequencer_set_valid = sequence && *sequence >= 0;
        msg.sequencer_set = msg.sequencer_set_valid ? static_cast<uint64_t>(*sequence) : 0;

        const auto lines = readInteger(parser, buffer, lines_);
        msg.line_status_valid = lines.has_value();
        // Aravis returns gint64 even for an unsigned GenICam register. Preserve every bit.
        msg.line_status = static_cast<uint64_t>(lines.value_or(0));
    }

  private:
    static const char* integerFeature(ArvGc* genicam, const char* name)
    {
        return genicam && ARV_IS_GC_INTEGER(arv_gc_get_node(genicam, name)) ? name : nullptr;
    }

    static std::optional<int64_t> readInteger(ArvChunkParser* parser, ArvBuffer* buffer,
                                              const char* name)
    {
        if (!parser || !name)
            return std::nullopt;
        GuardedGError err;
        const auto value = arv_chunk_parser_get_integer_value(parser, buffer, name, err.ref());
        if (err)
            return std::nullopt;
        return value;
    }

    const char* sequence_;
    const char* lines_;
};

}  // namespace camera_aravis2

#endif  // CAMERA_ARAVIS2__FRAME_IO_CHUNKS_H_
