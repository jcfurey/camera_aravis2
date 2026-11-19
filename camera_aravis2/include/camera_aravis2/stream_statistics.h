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

#ifndef CAMERA_ARAVIS2__STREAM_STATISTICS_H_
#define CAMERA_ARAVIS2__STREAM_STATISTICS_H_

#include <glib.h>

#include <chrono>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace camera_aravis2
{

struct StreamCounters
{
    guint64 completed = 0;
    guint64 failures = 0;
    guint64 underruns = 0;
    guint64 resent = 0;
    guint64 missing = 0;
    guint64 published = 0;
    guint64 dropped = 0;
    guint64 timestamp_drops = 0;

    diagnostic_msgs::msg::DiagnosticStatus describeInterval(
      const StreamCounters& previous, std::chrono::duration<double> interval,
      bool acquisition_requested, bool is_gige) const;
};

}  // namespace camera_aravis2

#endif  // CAMERA_ARAVIS2__STREAM_STATISTICS_H_
