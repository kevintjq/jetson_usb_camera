#include "frame_copy.h"
#include "frame_timing.h"

#include <camera_info_manager/camera_info_manager.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
using SteadyClock = std::chrono::steady_clock;

struct SampleUnref { void operator()(GstSample* sample) const { gst_sample_unref(sample); } };
using SamplePtr = std::unique_ptr<GstSample, SampleUnref>;

// Unmap even if image allocation/publishing throws.
struct MappedFrame
{
  GstVideoFrame frame = {};
  bool mapped = false;
  ~MappedFrame() { if (mapped) gst_video_frame_unmap(&frame); }
};

template<class T>
void readParameter(const ros::NodeHandle& pnh, const std::string& name, T& value)
{
  if (pnh.hasParam(name) && !pnh.getParam(name, value))
    throw std::runtime_error("Invalid parameter type: " + name);
}

template<class Values>
bool allFinite(const Values& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}
} // namespace

class JetsonGstCamera
{
public:
  JetsonGstCamera()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    loadParameters(pnh);
    image_pub_ = nh.advertise<sensor_msgs::Image>(image_topic_, 1);
    info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(info_topic_, 1);
    if (!camera_info_url_.empty()) {
      info_manager_.reset(new camera_info_manager::CameraInfoManager(nh, camera_name_, camera_info_url_));
      if (!info_manager_->isCalibrated())
        throw std::runtime_error("Cannot load calibrated CameraInfo: " + camera_info_url_);
    }
    gst_init(nullptr, nullptr);
    ROS_INFO("Jetson USB camera: %s, %dx%d @ %d FPS, output=%s", device_.c_str(),
             width_, height_, framerate_, output_encoding_.c_str());
    ROS_INFO("Image: %s; CameraInfo: %s; frame=%s", image_topic_.c_str(), info_topic_.c_str(), frame_id_.c_str());
  }

  ~JetsonGstCamera() { stop(); }

  void spin()
  {
    auto last_valid_frame = SteadyClock::now();
    SteadyClock::time_point retry_started;
    while (ros::ok()) {
      if (!pipeline_) {
        // Short sleeps let ROS service callbacks/shutdown run while reconnecting.
        if (retry_started != SteadyClock::time_point() &&
            std::chrono::duration<double>(SteadyClock::now() - retry_started).count() < reconnect_interval_) {
          ros::spinOnce();
          ros::WallDuration(0.01).sleep();
          continue;
        }
        if (!createPipeline()) {
          retry_started = SteadyClock::now();
          ros::spinOnce();
          continue;
        }
        last_valid_frame = SteadyClock::now();
      }
      if (!checkPipelineMessages()) {
        retry_started = SteadyClock::now();
        continue;
      }
      SamplePtr sample(gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), GST_SECOND / framerate_));
      if (sample && processSample(sample.get())) last_valid_frame = SteadyClock::now();
      sample.reset(); // Release decoder buffers before a possible pipeline stop.
      // Invalid samples must not keep an unusable stream alive indefinitely.
      if (std::chrono::duration<double>(SteadyClock::now() - last_valid_frame).count() >= frame_timeout_) {
        ROS_ERROR("No usable camera frame for %.2f s; restarting pipeline", frame_timeout_);
        stop();
        retry_started = SteadyClock::now();
      }
      ros::spinOnce();
    }
    stop();
  }

private:
  void loadParameters(const ros::NodeHandle& pnh)
  {
    readParameter(pnh, "device", device_);
    readParameter(pnh, "width", width_);
    readParameter(pnh, "height", height_);
    readParameter(pnh, "framerate", framerate_);
    readParameter(pnh, "frame_id", frame_id_);
    readParameter(pnh, "image_topic", image_topic_);
    readParameter(pnh, "camera_name", camera_name_);
    readParameter(pnh, "camera_info_url", camera_info_url_);
    const auto slash = image_topic_.find_last_of('/');
    info_topic_ = slash == std::string::npos ? image_topic_ + "/camera_info"
                                           : image_topic_.substr(0, slash) + "/camera_info";
    readParameter(pnh, "camera_info_topic", info_topic_);
    readParameter(pnh, "output_encoding", output_encoding_);
    readParameter(pnh, "reconnect_interval", reconnect_interval_);
    readParameter(pnh, "frame_timeout", frame_timeout_);
    readParameter(pnh, "max_frame_age", max_frame_age_);
    if (device_.empty() || frame_id_.empty() || image_topic_.empty() ||
        info_topic_.empty() || camera_name_.empty() || width_ <= 0 || height_ <= 0 || framerate_ <= 0 ||
        static_cast<uint64_t>(width_) * 4 > std::numeric_limits<uint32_t>::max())
      throw std::runtime_error("Invalid camera dimensions, rate, device or topic parameters");
    for (double value : {reconnect_interval_, frame_timeout_, max_frame_age_})
      if (!std::isfinite(value) || value <= 0.0)
        throw std::runtime_error("Camera timeouts must be finite and positive");
    if (output_encoding_ != "bgr8" && output_encoding_ != "rgba8")
      throw std::runtime_error("output_encoding must be bgr8 or rgba8");
  }

  bool createPipeline()
  {
    gchar* escaped = g_strescape(device_.c_str(), nullptr);
    const std::string device = escaped;
    g_free(escaped);
    const std::string format = output_encoding_ == "bgr8" ? "BGRx" : "RGBA";
    // VIC conversion into system memory; BGR packing is fused with the ROS copy.
    const std::string description = "v4l2src device=\"" + device +
        "\" ! image/jpeg,width=" + std::to_string(width_) + ",height=" + std::to_string(height_) +
        ",framerate=" + std::to_string(framerate_) + "/1 ! jpegparse ! nvv4l2decoder ! nvvidconv ! "
        "video/x-raw,format=" + format + " ! appsink name=camera_sink sync=false max-buffers=1 "
        "drop=true enable-last-sample=false";
    ROS_INFO("GStreamer pipeline: %s", description.c_str());
    GError* error = nullptr;
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (error || !pipeline_) {
      ROS_ERROR("Cannot create pipeline: %s", error ? error->message : "unknown error");
      g_clear_error(&error);
      stop();
      return false;
    }
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "camera_sink");
    bus_ = gst_element_get_bus(pipeline_);
    if (!appsink_ || !bus_ || gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      ROS_ERROR("Cannot start camera pipeline");
      stop();
      return false;
    }
    last_stamp_ = ros::Time();
    last_running_time_ = GST_CLOCK_TIME_NONE;
    return true;
  }

  bool checkPipelineMessages()
  {
    GstMessage* message = gst_bus_pop_filtered(bus_, static_cast<GstMessageType>(
        GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    if (!message) return true;
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError* error = nullptr;
      gchar* debug = nullptr;
      gst_message_parse_error(message, &error, &debug);
      ROS_ERROR("GStreamer error: %s (%s)", error ? error->message : "unknown", debug ? debug : "");
      g_clear_error(&error);
      g_free(debug);
    } else ROS_ERROR("Camera stream reached EOS");
    gst_message_unref(message);
    stop();
    return false;
  }

  bool captureStamp(GstSample* sample, GstBuffer* buffer, ros::Time& stamp, GstClockTime& capture_running)
  {
    const GstSegment* segment = gst_sample_get_segment(sample);
    if (!segment || segment->format != GST_FORMAT_TIME || !GST_BUFFER_PTS_IS_VALID(buffer)) return false;
    const GstClockTime running = gst_segment_to_running_time(segment, GST_FORMAT_TIME, GST_BUFFER_PTS(buffer));
    if (!GST_CLOCK_TIME_IS_VALID(running) ||
        (GST_CLOCK_TIME_IS_VALID(last_running_time_) && running <= last_running_time_)) return false;
    capture_running = running;
    const GstClockTime base = gst_element_get_base_time(pipeline_);
    GstClock* clock = gst_element_get_clock(pipeline_);
    if (!clock) return false;
    const GstClockTime clock_now = gst_clock_get_time(clock);
    gst_object_unref(clock);
    const ros::Time ros_now = ros::Time::now();
    double age = 0.0;
    if (!GST_CLOCK_TIME_IS_VALID(running) || !GST_CLOCK_TIME_IS_VALID(base) ||
        !GST_CLOCK_TIME_IS_VALID(clock_now) || ros_now.isZero() ||
        !camera_frames::frameAge(clock_now, base, running, max_frame_age_, age) || ros_now.toSec() <= age)
      return false;
    stamp = ros_now - ros::Duration(age);
    // Backwards ROS clock changes or duplicate driver timestamps are rejected.
    // A persistent discontinuity will cause the normal usable-frame timeout/restart.
    return last_stamp_.isZero() || stamp > last_stamp_;
  }

  bool processSample(GstSample* sample)
  {
    GstCaps* caps = gst_sample_get_caps(sample);
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstVideoInfo info;
    const GstVideoFormat expected = output_encoding_ == "bgr8" ? GST_VIDEO_FORMAT_BGRx : GST_VIDEO_FORMAT_RGBA;
    if (!caps || !buffer || GST_BUFFER_FLAG_IS_SET(buffer, GST_BUFFER_FLAG_CORRUPTED) ||
        !gst_video_info_from_caps(&info, caps) || GST_VIDEO_INFO_FORMAT(&info) != expected) {
      ROS_WARN_THROTTLE(5.0, "Invalid sample caps, buffer or video format");
      return false;
    }
    ros::Time stamp;
    GstClockTime capture_running = GST_CLOCK_TIME_NONE;
    if (!captureStamp(sample, buffer, stamp, capture_running)) {
      ROS_WARN_THROTTLE(5.0, "Missing, stale or discontinuous capture timestamp; dropping frame");
      return false;
    }
    MappedFrame mapped;
    mapped.mapped = gst_video_frame_map(&mapped.frame, &info, buffer, GST_MAP_READ);
    if (!mapped.mapped) {
      ROS_WARN_THROTTLE(5.0, "Cannot map decoded video frame");
      return false;
    }
    const int width = GST_VIDEO_FRAME_WIDTH(&mapped.frame);
    const int height = GST_VIDEO_FRAME_HEIGHT(&mapped.frame);
    const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&mapped.frame, 0);
    const auto* source = static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&mapped.frame, 0));
    const std::size_t channels = output_encoding_ == "bgr8" ? 3 : 4;
    if (width <= 0 || height <= 0 || !source || stride <= 0 ||
        static_cast<uint64_t>(width) * 4 > static_cast<uint64_t>(stride) ||
        static_cast<uint64_t>(width) * channels > std::numeric_limits<uint32_t>::max() ||
        static_cast<uint64_t>(height) > std::numeric_limits<std::size_t>::max() / (static_cast<std::size_t>(width) * channels)) {
      ROS_WARN_THROTTLE(5.0, "Invalid decoded frame layout");
      return false;
    }
    if (image_pub_.getNumSubscribers() > 0) {
      // Reuse storage only when neither ROS nor an in-process consumer owns it.
      if (!image_message_ || !image_message_.unique()) image_message_.reset(new sensor_msgs::Image());
      auto& image = *image_message_;
      image.header.stamp = stamp;
      image.header.frame_id = frame_id_;
      image.width = width;
      image.height = height;
      image.encoding = output_encoding_;
      image.is_bigendian = false;
      image.step = static_cast<uint32_t>(static_cast<std::size_t>(width) * channels);
      image.data.resize(static_cast<std::size_t>(image.step) * height);
      if (output_encoding_ == "bgr8")
        camera_frames::bgrxToBgr(source, stride, image.data.data(), width, height);
      else camera_frames::copyRgba(source, stride, image.data.data(), width, height);
      image_pub_.publish(image_message_);
    }
    if (info_pub_.getNumSubscribers() > 0) publishCameraInfo(stamp, width, height);
    last_stamp_ = stamp;
    last_running_time_ = capture_running;
    return true;
  }

  void publishCameraInfo(const ros::Time& stamp, uint32_t width, uint32_t height)
  {
    sensor_msgs::CameraInfo info;
    if (info_manager_) {
      info = info_manager_->getCameraInfo();
      const bool valid = info.width == width && info.height == height &&
          allFinite(info.K) && allFinite(info.R) && allFinite(info.P) && allFinite(info.D) &&
          info.K[0] > 0.0 && info.K[4] > 0.0 && std::abs(info.K[8] - 1.0) < 1e-6 &&
          info.P[0] > 0.0 && info.P[5] > 0.0;
      if (!valid) {
        ROS_WARN_THROTTLE(5.0, "CameraInfo invalid or mismatched (%ux%u vs %ux%u); publishing UNCALIBRATED info",
                          info.width, info.height, width, height);
        info = sensor_msgs::CameraInfo(); // K[0]=0: ROS uncalibrated-camera convention.
      }
    }
    info.width = width;
    info.height = height;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info_pub_.publish(info);
  }

  void stop()
  {
    if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (appsink_) gst_object_unref(appsink_);
    if (bus_) gst_object_unref(bus_);
    if (pipeline_) gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    appsink_ = nullptr;
    bus_ = nullptr;
    last_stamp_ = ros::Time();
    last_running_time_ = GST_CLOCK_TIME_NONE;
  }

  GstElement* pipeline_ = nullptr;
  GstElement* appsink_ = nullptr;
  GstBus* bus_ = nullptr;
  ros::Publisher image_pub_;
  ros::Publisher info_pub_;
  sensor_msgs::ImagePtr image_message_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  ros::Time last_stamp_;
  GstClockTime last_running_time_ = GST_CLOCK_TIME_NONE;
  std::string device_ = "/dev/video0";
  std::string frame_id_ = "camera";
  std::string image_topic_ = "/camera/image_raw";
  std::string info_topic_;
  std::string camera_name_ = "camera";
  std::string camera_info_url_;
  std::string output_encoding_ = "bgr8";
  int width_ = 1280;
  int height_ = 720;
  int framerate_ = 60;
  double reconnect_interval_ = 2.0;
  double frame_timeout_ = 3.0;
  double max_frame_age_ = 0.5;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jetson_gst_camera");
  try {
    JetsonGstCamera camera;
    camera.spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Camera node failed: %s", error.what());
    return 1;
  }
  return 0;
}
