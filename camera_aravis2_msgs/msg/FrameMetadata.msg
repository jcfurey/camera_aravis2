# Matches the Image and CameraInfo from this buffer, including any ROS timestamp offset.
std_msgs/Header header

# Same camera identifier as the corresponding stream status on /diagnostics.
string hardware_id
# Transport frame ID reported by Aravis. It can wrap or reset between acquisitions.
uint64 frame_id
# Buffer timestamp in Aravis-normalized nanoseconds. Its epoch depends on the camera.
uint64 device_timestamp_ns
# Host reception time recorded by Aravis, in nanoseconds.
uint64 host_receive_timestamp_ns
# Pixel Format Naming Convention (PFNC) code in the received image, before ROS conversion.
uint32 pixel_format

# True when Aravis reports chunk data in this buffer.
bool chunks_present
# Values come from this frame's ChunkExposureTime and ChunkGain. Missing or unreadable
# values are NaN with the corresponding valid flag false. Units are from the camera XML.
bool exposure_valid
float64 exposure_time
string exposure_unit
bool gain_valid
float64 gain
string gain_unit

# Active sequencer set and digital-line status reported in this frame's chunks.
# Check each valid flag independently. Both fields read zero when unavailable.
# Set zero and an all-low line mask are also valid camera readings.
bool sequencer_set_valid
uint64 sequencer_set
bool line_status_valid
uint64 line_status

# Inclusive limits from this buffer's ChunkPixelDynamicRangeMin and ChunkPixelDynamicRangeMax.
# Units are camera pixel counts before ROS conversion. A valid pair satisfies 0 <= min < max.
# If either endpoint is missing or the pair is invalid, both values are zero.
bool pixel_dynamic_range_valid
uint64 pixel_dynamic_range_min
uint64 pixel_dynamic_range_max

# Configuration sampled before capture. Zero bit depths indicate unknown precision.
string pixel_format_name
uint8 pixel_bit_depth
uint8 storage_bit_depth
# Number of left-shift bits applied by the driver to unsigned measurement samples.
uint8 sample_shift
bool adc_bit_depth_valid
uint8 adc_bit_depth
bool black_level_valid
float64 black_level
string black_level_unit
string black_level_selector
# Inclusive source counts. Source is chunk, camera, pixel_format or unavailable.
string intensity_range_source
uint64 intensity_min
uint64 intensity_max
# Observed lock covers the reception time of this frame.
bool ptp_locked
# Buffer ROI origin and acquisition-time sampling configuration.
uint32 roi_offset_x
uint32 roi_offset_y
bool geometry_valid
uint32 binning_x
uint32 binning_y
uint32 decimation_x
uint32 decimation_y
bool reverse_x
bool reverse_y
