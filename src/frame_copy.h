#ifndef JETSON_GST_CAMERA_FRAME_COPY_H
#define JETSON_GST_CAMERA_FRAME_COPY_H

#include <cstddef>
#include <cstdint>
#include <cstring>

#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace camera_frames
{
// Pack directly into the ROS message, without an intermediate BGR image.
inline void bgrxToBgr(const uint8_t* source, std::size_t source_stride,
                      uint8_t* destination, std::size_t width, std::size_t height)
{
  for (std::size_t row = 0; row < height; ++row) {
    const uint8_t* input = source + row * source_stride;
    uint8_t* output = destination + row * width * 3;
    std::size_t column = 0;
#if defined(__aarch64__) || defined(__ARM_NEON)
    for (; column + 16 <= width; column += 16) {
      const uint8x16x4_t pixels = vld4q_u8(input + column * 4);
      uint8x16x3_t bgr;
      bgr.val[0] = pixels.val[0];
      bgr.val[1] = pixels.val[1];
      bgr.val[2] = pixels.val[2];
      vst3q_u8(output + column * 3, bgr);
    }
#endif
    for (; column < width; ++column) {
      output[column * 3] = input[column * 4];
      output[column * 3 + 1] = input[column * 4 + 1];
      output[column * 3 + 2] = input[column * 4 + 2];
    }
  }
}

inline void copyPacked(const uint8_t* source, std::size_t source_stride,
                       uint8_t* destination, std::size_t row_bytes, std::size_t height)
{
  if (source_stride == row_bytes) {
    std::memcpy(destination, source, row_bytes * height);
    return;
  }
  for (std::size_t row = 0; row < height; ++row)
    std::memcpy(destination + row * row_bytes, source + row * source_stride, row_bytes);
}
} // namespace camera_frames
#endif
