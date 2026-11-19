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
