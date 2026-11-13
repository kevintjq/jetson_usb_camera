#ifndef JETSON_GST_CAMERA_FRAME_TIMING_H
#define JETSON_GST_CAMERA_FRAME_TIMING_H

#include <cstdint>

namespace camera_frames
{
// GStreamer absolute capture clock = pipeline base-time + segment running-time.
// The small future allowance only covers clock read/driver jitter; never extend age.
inline bool frameAge(uint64_t clock_now, uint64_t base_time, uint64_t running_time,
                     double max_age, double& age)
{
  if (clock_now < base_time) return false;
  const uint64_t elapsed = clock_now - base_time;
  if (running_time > elapsed) {
    if (running_time - elapsed > 10000000ULL) return false; // 10 ms
    age = 0.0;
  } else {
    age = static_cast<double>(elapsed - running_time) * 1e-9;
  }
  return age <= max_age;
}
} // namespace camera_frames
#endif
