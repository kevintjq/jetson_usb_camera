#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <cstring>

class JetsonGstCamera
{
public:
    JetsonGstCamera()
        : pipeline_(nullptr),
          appsink_(nullptr),
          bus_(nullptr),
          running_(false),
          configured_(false)
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        pnh.param<std::string>(
            "device",
            device_,
            "/dev/video0");

        pnh.param<int>(
            "width",
            width_,
            1280);

        pnh.param<int>(
            "height",
            height_,
            720);

        pnh.param<int>(
            "framerate",
            framerate_,
            60);

        pnh.param<std::string>(
            "frame_id",
            frame_id_,
            "camera");

        pnh.param<std::string>(
            "image_topic",
            image_topic_,
            "/camera/image_raw");

        pnh.param<std::string>(
            "camera_name",
            camera_name_,
            "camera");

        pnh.param<std::string>(
            "camera_info_url",
            camera_info_url_,
            "");

        std::string default_camera_info_topic = image_topic_;
        const std::string::size_type last_slash =
            default_camera_info_topic.find_last_of('/');

        if (last_slash != std::string::npos)
        {
            default_camera_info_topic =
                default_camera_info_topic.substr(0, last_slash) +
                "/camera_info";
        }
        else
        {
            default_camera_info_topic += "/camera_info";
        }

        pnh.param<std::string>(
            "camera_info_topic",
            camera_info_topic_,
            default_camera_info_topic);

        pnh.param<double>(
            "reconnect_interval",
            reconnect_interval_,
            2.0);

        pnh.param<double>(
            "frame_timeout",
            frame_timeout_,
            3.0);

        if (device_.empty() ||
            width_ <= 0 ||
            height_ <= 0 ||
            framerate_ <= 0 ||
            reconnect_interval_ <= 0.0 ||
            frame_timeout_ <= 0.0 ||
            frame_id_.empty() ||
            image_topic_.empty() ||
            camera_name_.empty() ||
            camera_info_topic_.empty())
        {
            ROS_ERROR(
                "Invalid camera parameters: device='%s', width=%d, height=%d, framerate=%d, reconnect_interval=%.2f, frame_timeout=%.2f, frame_id='%s', image_topic='%s', camera_name='%s', camera_info_topic='%s'",
                device_.c_str(),
                width_,
                height_,
                framerate_,
                reconnect_interval_,
                frame_timeout_,
                frame_id_.c_str(),
                image_topic_.c_str(),
                camera_name_.c_str(),
                camera_info_topic_.c_str());
            return;
        }

        image_pub_ = nh.advertise<sensor_msgs::Image>(
            image_topic_,
            2);

        camera_info_pub_ =
            nh.advertise<sensor_msgs::CameraInfo>(
                camera_info_topic_,
                2);

        camera_info_manager_.reset(
            new camera_info_manager::CameraInfoManager(
                nh,
                camera_name_,
                camera_info_url_));

        if (!camera_info_url_.empty() &&
            !camera_info_manager_->isCalibrated())
        {
            ROS_WARN(
                "Camera calibration could not be loaded from URL: %s",
                camera_info_url_.c_str());
        }

        ROS_INFO("========================================");
        ROS_INFO(" Jetson GStreamer Camera");
        ROS_INFO("========================================");
        ROS_INFO("device     : %s", device_.c_str());
        ROS_INFO("resolution : %dx%d", width_, height_);
        ROS_INFO("framerate  : %d FPS", framerate_);
        ROS_INFO("frame_id   : %s", frame_id_.c_str());
        ROS_INFO("topic      : %s", image_topic_.c_str());
        ROS_INFO("camera info: %s", camera_info_topic_.c_str());
        ROS_INFO("reconnect  : %.2f seconds", reconnect_interval_);
        ROS_INFO("frame timeout: %.2f seconds", frame_timeout_);
        ROS_INFO("========================================");

        gst_init(nullptr, nullptr);

        configured_ = true;

        if (!createPipeline())
        {
            ROS_WARN(
                "Initial camera pipeline could not be started. "
                "Automatic reconnect will retry every %.2f seconds.",
                reconnect_interval_);
        }
    }

    ~JetsonGstCamera()
    {
        stop();
    }

    bool isRunning() const
    {
        return running_;
    }

    bool isConfigured() const
    {
        return configured_;
    }

    void spin()
    {
        ros::WallRate reconnect_rate(1.0 / reconnect_interval_);
        std::chrono::steady_clock::time_point last_frame_time =
            std::chrono::steady_clock::now();

        while (ros::ok())
        {
            if (!running_)
            {
                if (!createPipeline())
                {
                    ros::spinOnce();
                    reconnect_rate.sleep();
                    continue;
                }

                last_frame_time =
                    std::chrono::steady_clock::now();
            }

            if (!checkPipelineMessages())
            {
                ros::spinOnce();
                reconnect_rate.sleep();
                continue;
            }

            GstSample *sample =
                gst_app_sink_try_pull_sample(
                    GST_APP_SINK(appsink_),
                    GST_SECOND / framerate_);

            if (sample != nullptr)
            {
                processSample(sample);
                gst_sample_unref(sample);
                last_frame_time =
                    std::chrono::steady_clock::now();
            }
            else
            {
                const std::chrono::duration<double> elapsed =
                    std::chrono::steady_clock::now() -
                    last_frame_time;

                if (elapsed.count() >= frame_timeout_)
                {
                    ROS_ERROR(
                        "No camera frame received for %.2f seconds. "
                        "Restarting the GStreamer pipeline.",
                        elapsed.count());
                    stop();
                    ros::spinOnce();
                    reconnect_rate.sleep();
                    continue;
                }
            }

            ros::spinOnce();
        }

        stop();
    }

private:

    bool checkPipelineMessages()
    {
        if (!bus_)
            return false;

        GstMessage *message =
            gst_bus_pop_filtered(
                bus_,
                static_cast<GstMessageType>(
                    GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

        if (!message)
            return true;

        if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR)
        {
            GError *error = nullptr;
            gchar *debug = nullptr;

            gst_message_parse_error(
                message,
                &error,
                &debug);

            ROS_ERROR(
                "GStreamer runtime error: %s",
                error ? error->message : "unknown error");

            if (debug)
                ROS_ERROR("GStreamer debug details: %s", debug);

            g_clear_error(&error);
            g_free(debug);
        }
        else
        {
            ROS_ERROR("GStreamer pipeline reached EOS.");
        }

        gst_message_unref(message);
        stop();
        return false;
    }

    bool createPipeline()
    {
        /*
         * Pipeline:
         *
         * v4l2src
         *   ↓
         * MJPEG
         *   ↓
         * jpegparse
         *   ↓
         * nvv4l2decoder       NVIDIA hardware JPEG decoder
         *   ↓
         * NVMM
         *   ↓
         * nvvidconv           NVIDIA conversion
         *   ↓
         * videoconvert        CPU/system-memory conversion
         *   ↓
         * BGR
         *   ↓
         * appsink
         */

        std::string pipeline_description =
            "v4l2src device=" + device_ +
            " ! image/jpeg,width=" +
            std::to_string(width_) +
            ",height=" +
            std::to_string(height_) +
            ",framerate=" +
            std::to_string(framerate_) +
            "/1 ! "
            "jpegparse ! "
            "nvv4l2decoder ! "
            "nvvidconv ! "
            "video/x-raw,format=BGRx ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink name=appsink "
            "sync=false "
            "max-buffers=1 "
            "drop=true "
            "enable-last-sample=false";

        ROS_INFO("GStreamer pipeline:");
        ROS_INFO("%s", pipeline_description.c_str());

        GError *error = nullptr;

        pipeline_ = gst_parse_launch(
            pipeline_description.c_str(),
            &error);

        if (pipeline_ == nullptr)
        {
            if (error)
            {
                ROS_ERROR(
                    "GStreamer pipeline error: %s",
                    error->message);

                g_error_free(error);
            }

            return false;
        }

        if (error)
        {
            ROS_ERROR(
                "GStreamer parse error: %s",
                error->message);

            g_error_free(error);
            error = nullptr;

            gst_object_unref(pipeline_);
            pipeline_ = nullptr;

            return false;
        }

        appsink_ = gst_bin_get_by_name(
            GST_BIN(pipeline_),
            "appsink");

        if (appsink_ == nullptr)
        {
            ROS_ERROR("Cannot find appsink.");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;

            return false;
        }

        bus_ = gst_element_get_bus(pipeline_);

        if (bus_ == nullptr)
        {
            ROS_ERROR("Cannot get GStreamer bus.");
            stop();
            return false;
        }

        /*
         * We want raw BGR frames.
         */
        gst_app_sink_set_emit_signals(
            GST_APP_SINK(appsink_),
            false);

        gst_app_sink_set_drop(
            GST_APP_SINK(appsink_),
            true);

        gst_app_sink_set_max_buffers(
            GST_APP_SINK(appsink_),
            1);

        GstStateChangeReturn ret =
            gst_element_set_state(
                pipeline_,
                GST_STATE_PLAYING);

        if (ret == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR(
                "Failed to set GStreamer pipeline to PLAYING.");

            stop();
            return false;
        }

        ROS_INFO("GStreamer pipeline started.");

        running_ = true;
        return true;
    }

    void processSample(GstSample *sample)
    {
        if (!sample)
            return;

        GstCaps *caps =
            gst_sample_get_caps(sample);

        if (!caps)
            return;

        GstVideoInfo video_info;
        if (!gst_video_info_from_caps(&video_info, caps))
        {
            ROS_WARN_THROTTLE(
                5.0,
                "Failed to read video information from GStreamer caps.");
            return;
        }

        if (GST_VIDEO_INFO_FORMAT(&video_info) != GST_VIDEO_FORMAT_BGR)
        {
            ROS_WARN_THROTTLE(
                5.0,
                "Unexpected GStreamer format: %s",
                gst_video_format_to_string(
                    GST_VIDEO_INFO_FORMAT(&video_info)));
            return;
        }

        GstBuffer *buffer =
            gst_sample_get_buffer(sample);

        if (!buffer)
            return;

        GstVideoFrame frame = GST_VIDEO_FRAME_INIT;
        if (!gst_video_frame_map(
                &frame,
                &video_info,
                buffer,
                GST_MAP_READ))
        {
            ROS_WARN_THROTTLE(
                5.0,
                "Failed to map GStreamer video frame.");
            return;
        }

        const int width = GST_VIDEO_FRAME_WIDTH(&frame);
        const int height = GST_VIDEO_FRAME_HEIGHT(&frame);
        const size_t row_bytes =
            static_cast<size_t>(width) * 3;
        const guint8 *source =
            GST_VIDEO_FRAME_PLANE_DATA(&frame, 0);
        const gint source_stride =
            GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);

        if (width <= 0 ||
            height <= 0 ||
            source == nullptr ||
            source_stride <= 0 ||
            static_cast<size_t>(source_stride) < row_bytes)
        {
            ROS_WARN_THROTTLE(
                5.0,
                "Invalid mapped video frame layout.");

            gst_video_frame_unmap(&frame);
            return;
        }

        sensor_msgs::ImagePtr msg =
            boost::make_shared<sensor_msgs::Image>();

        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = frame_id_;

        msg->height = height;
        msg->width = width;

        msg->encoding = "bgr8";
        msg->is_bigendian = false;

        msg->step = static_cast<uint32_t>(row_bytes);

        msg->data.resize(
            row_bytes * static_cast<size_t>(height));

        for (int row = 0; row < height; ++row)
        {
            std::memcpy(
                msg->data.data() +
                    static_cast<size_t>(row) * row_bytes,
                source +
                    static_cast<size_t>(row) *
                        static_cast<size_t>(source_stride),
                row_bytes);
        }

        gst_video_frame_unmap(&frame);

        image_pub_.publish(msg);

        if (camera_info_manager_)
        {
            sensor_msgs::CameraInfo camera_info =
                camera_info_manager_->getCameraInfo();

            camera_info.header = msg->header;

            if (camera_info.width == 0)
                camera_info.width = msg->width;

            if (camera_info.height == 0)
                camera_info.height = msg->height;

            if (camera_info.width != msg->width ||
                camera_info.height != msg->height)
            {
                ROS_WARN_THROTTLE(
                    5.0,
                    "CameraInfo size %ux%u differs from image size %ux%u.",
                    camera_info.width,
                    camera_info.height,
                    msg->width,
                    msg->height);
            }

            camera_info_pub_.publish(camera_info);
        }
    }

    void stop()
    {
        running_ = false;

        if (pipeline_)
        {
            gst_element_set_state(
                pipeline_,
                GST_STATE_NULL);
        }

        if (appsink_)
        {
            gst_object_unref(appsink_);
            appsink_ = nullptr;
        }

        if (bus_)
        {
            gst_object_unref(bus_);
            bus_ = nullptr;
        }

        if (pipeline_)
        {
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }
    }

private:

    GstElement *pipeline_;
    GstElement *appsink_;
    GstBus *bus_;

    bool running_;
    bool configured_;

    std::string device_;
    std::string frame_id_;
    std::string image_topic_;
    std::string camera_name_;
    std::string camera_info_url_;
    std::string camera_info_topic_;

    int width_;
    int height_;
    int framerate_;
    double reconnect_interval_;
    double frame_timeout_;

    ros::Publisher image_pub_;
    ros::Publisher camera_info_pub_;
    std::unique_ptr<camera_info_manager::CameraInfoManager>
        camera_info_manager_;
};


int main(int argc, char **argv)
{
    ros::init(
        argc,
        argv,
        "jetson_gst_camera");

    JetsonGstCamera camera;

    if (!camera.isConfigured())
    {
        ROS_ERROR("Camera configuration is invalid.");
        return 1;
    }

    camera.spin();

    return 0;
}
