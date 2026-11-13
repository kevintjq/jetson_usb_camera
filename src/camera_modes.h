#ifndef JETSON_GST_CAMERA_MODES_H
#define JETSON_GST_CAMERA_MODES_H

#include <string>
#include <vector>

namespace camera_modes
{
struct InputMode
{
  std::string name;
  std::string caps;
  std::string parser;
  std::string software_decoder;
  bool raw;
  bool vic_input;
};

inline std::vector<InputMode> supportedInputs()
{
  std::vector<InputMode> modes = {
    {"MJPEG", "image/jpeg", "jpegparse", "jpegdec", false, false},
    {"H264", "video/x-h264", "h264parse", "avdec_h264", false, false},
    {"H265", "video/x-h265", "h265parse", "avdec_h265", false, false}
  };
  const char* raw_formats[] = {"BGR", "BGRx", "RGBA", "RGB", "RGBx", "BGRA", "GRAY8",
      "YUY2", "UYVY", "YVYU", "NV12", "I420", "YV12", "NV21", "NV16", "NV24", "GRAY16_LE", "GRAY16_BE"};
  for (const char* format : raw_formats) {
    const std::string name = format;
    const bool vic = name == "BGRx" || name == "RGBA" || name == "GRAY8" ||
        name == "YUY2" || name == "UYVY" || name == "YVYU" || name == "NV12" ||
        name == "I420" || name == "NV16" || name == "NV24";
    modes.push_back({name, "video/x-raw,format=" + name, "", "", true, vic});
  }
  // Bayer requires bayer2rgb; do not mistake sensor data for ordinary RGB pixels.
  for (const char* pattern : {"bggr", "rggb", "grbg", "gbrg"})
    modes.push_back({std::string("Bayer-") + pattern, std::string("video/x-bayer,format=") + pattern,
                     "bayer2rgb", "", false, false});
  return modes;
}

struct Plan
{
  std::string key;
  std::string input_caps;
  std::string processing;
  std::string output_format;
  std::vector<std::string> plugins;
  int priority;
};

inline std::vector<Plan> plans(const InputMode& mode, const std::string& encoding)
{
  std::vector<Plan> result;
  const std::string hardware_format = encoding == "bgr8" ? "BGRx" : "RGBA";
  const std::string software_format = encoding == "bgr8" ? "BGR" : "RGBA";
  const bool bayer = mode.caps.find("video/x-bayer") == 0;
  const bool direct = mode.raw && (mode.name == software_format || mode.name == hardware_format);
  if (direct)
    result.push_back({mode.name + "/direct", mode.caps, "", mode.name, {}, 0});
  if (!direct && ((mode.raw && mode.vic_input) || (!mode.raw && !bayer))) {
    std::vector<std::string> plugins = {"nvvidconv"};
    std::string processing;
    if (!mode.raw) {
      processing = mode.parser + " ! nvv4l2decoder ! ";
      plugins.push_back(mode.parser);
      plugins.push_back("nvv4l2decoder");
    }
    processing += "nvvidconv ! ";
    const int priority = mode.raw ? 20 : (mode.name == "MJPEG" ? 10 : 30);
    result.push_back({mode.name + "/hardware", mode.caps, processing, hardware_format, plugins, priority});
  }
  if (mode.name == "MJPEG")
    result.push_back({"MJPEG/nvjpeg", mode.caps, "jpegparse ! nvjpegdec ! nvvidconv ! ",
                      hardware_format, {"jpegparse", "nvjpegdec", "nvvidconv"}, 11});
  if (!direct) {
    std::vector<std::string> plugins = {"videoconvert"};
    std::string processing;
    if (bayer) {
      processing = "bayer2rgb ! ";
      plugins.push_back("bayer2rgb");
    } else if (!mode.raw) {
      // jpegdec accepts complete V4L2 JPEG frames without jpegparse.
      if (mode.name != "MJPEG") { processing += mode.parser + " ! "; plugins.push_back(mode.parser); }
      processing += mode.software_decoder + " ! ";
      plugins.push_back(mode.software_decoder);
    }
    processing += "videoconvert ! ";
    // Without acceleration, try raw conversion before paying for JPEG decode.
    const int priority = mode.raw ? 40 : (mode.name == "MJPEG" ? 50 : 60);
    result.push_back({mode.name + "/software", mode.caps, processing, software_format, plugins, priority});
  }
  return result;
}
} // namespace camera_modes
#endif
