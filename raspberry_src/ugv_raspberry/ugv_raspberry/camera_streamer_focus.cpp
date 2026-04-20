// camera_streamer_focus.cpp
// ROS 2 node (C++) – camera stream + altitude-driven dynamic focus.
//
// Subscribes to /data (uav_msgs/UavTelemetry) and updates the RPi Camera
// Module 3 focus via V4L2 ioctls whenever barometric altitude changes by
// more than the configured deadband.
//
// Focus mapping is a configurable LINEAR interpolation:
//   alt_min_m -> focus_at_alt_min   (closer target, higher focus_absolute)
//   alt_max_m -> focus_at_alt_max   (farther target, lower focus_absolute)
// Values outside [alt_min, alt_max] are clamped.
//
// RPi Camera Module 3 lens range: focus_absolute 0 = infinity, 1000 = macro.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "uav_msgs/msg/uav_telemetry.hpp"

namespace {

constexpr int  kDefaultWidth        = 1280;
constexpr int  kDefaultHeight       = 720;
constexpr int  kDefaultFps          = 30;
constexpr int  kDefaultJpegQuality  = 60;
constexpr char kDefaultTopic[]      = "/ugv_camera/image_raw/compressed";
constexpr char kFrameId[]           = "camera_frame";
constexpr int  kQosDepth            = 5;

bool v4l2_set_focus(const std::string & dev, int value)
{
    int fd = ::open(dev.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) return false;

    v4l2_control ctrl{};
    ctrl.id    = V4L2_CID_FOCUS_AUTO;
    ctrl.value = 0;
    ::ioctl(fd, VIDIOC_S_CTRL, &ctrl);

    ctrl.id    = V4L2_CID_FOCUS_ABSOLUTE;
    ctrl.value = value;
    bool ok = (::ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0);

    ::close(fd);
    return ok;
}

}  // namespace

class CameraStreamerFocus : public rclcpp::Node {
public:
    CameraStreamerFocus() : Node("camera_streamer_focus")
    {
        declare_parameter("width",               kDefaultWidth);
        declare_parameter("height",              kDefaultHeight);
        declare_parameter("fps",                 kDefaultFps);
        declare_parameter("jpeg_quality",        kDefaultJpegQuality);
        declare_parameter("topic",               std::string(kDefaultTopic));
        declare_parameter("v4l2_subdev",         std::string("/dev/v4l-subdev0"));
        declare_parameter("alt_min_m",           0.5);
        declare_parameter("alt_max_m",           5.0);
        declare_parameter("focus_at_alt_min",    700);   // close -> high value
        declare_parameter("focus_at_alt_max",    150);   // far   -> low  value
        declare_parameter("focus_deadband_m",    0.30);

        const int w          = get_parameter("width").as_int();
        const int h          = get_parameter("height").as_int();
        const int fps        = get_parameter("fps").as_int();
        const int q          = get_parameter("jpeg_quality").as_int();
        topic_               = get_parameter("topic").as_string();
        subdev_              = get_parameter("v4l2_subdev").as_string();
        alt_min_m_           = get_parameter("alt_min_m").as_double();
        alt_max_m_           = get_parameter("alt_max_m").as_double();
        focus_at_alt_min_    = get_parameter("focus_at_alt_min").as_int();
        focus_at_alt_max_    = get_parameter("focus_at_alt_max").as_int();
        focus_deadband_m_    = get_parameter("focus_deadband_m").as_double();

        if (alt_max_m_ <= alt_min_m_) {
            throw std::runtime_error(
                "alt_max_m must be greater than alt_min_m");
        }

        gst_init(nullptr, nullptr);

        const std::string pipeline_str =
            "libcamerasrc af-mode=0 name=src ! "
            "video/x-raw,width="  + std::to_string(w) +
            ",height="            + std::to_string(h) +
            ",framerate="         + std::to_string(fps) + "/1,format=NV12 ! "
            "videoconvert ! "
            "jpegenc quality="    + std::to_string(q) + " ! "
            "appsink name=sink sync=false drop=true max-buffers=1 emit-signals=true";

        RCLCPP_INFO(get_logger(), "Pipeline: %s", pipeline_str.c_str());

        GError * err = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &err);
        if (!pipeline_ || err) {
            const std::string msg = err ? err->message : "unknown error";
            if (err) g_error_free(err);
            throw std::runtime_error("GStreamer parse_launch: " + msg);
        }

        sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!sink_) throw std::runtime_error("appsink 'sink' not found");

        g_signal_connect(sink_, "new-sample",
            G_CALLBACK(CameraStreamerFocus::on_new_sample_static), this);

        auto qos = rclcpp::QoS(kQosDepth)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(topic_, qos);

        sub_telemetry_ = create_subscription<uav_msgs::msg::UavTelemetry>(
            "/data", rclcpp::QoS(5).best_effort(),
            [this](const uav_msgs::msg::UavTelemetry::SharedPtr msg) {
                on_telemetry(msg);
            });

        apply_focus(alt_min_m_);  // sensible default before first telemetry tick

        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING)
                == GST_STATE_CHANGE_FAILURE) {
            throw std::runtime_error("Failed to start GStreamer pipeline");
        }

        bus_thread_ = std::thread([this] { run_bus_loop(); });

        RCLCPP_INFO(get_logger(),
            "CameraStreamerFocus: %dx%d @ %d fps  JPEG q=%d  topic=%s  subdev=%s",
            w, h, fps, q, topic_.c_str(), subdev_.c_str());
        RCLCPP_INFO(get_logger(),
            "Focus map: %.2fm->%d, %.2fm->%d  deadband=%.2fm",
            alt_min_m_, focus_at_alt_min_,
            alt_max_m_, focus_at_alt_max_,
            focus_deadband_m_);
    }

    ~CameraStreamerFocus() override
    {
        bus_running_ = false;
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
        if (sink_) gst_object_unref(sink_);
        if (bus_thread_.joinable()) bus_thread_.join();
    }

private:
    GstElement *       pipeline_{nullptr};
    GstElement *       sink_{nullptr};
    std::thread        bus_thread_;
    std::atomic<bool>  bus_running_{true};

    std::string        topic_;
    std::string        subdev_;
    double             alt_min_m_{0.5};
    double             alt_max_m_{5.0};
    int                focus_at_alt_min_{700};
    int                focus_at_alt_max_{150};
    double             focus_deadband_m_{0.30};

    double             last_focus_alt_m_{-999.0};
    std::mutex         focus_mutex_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
    rclcpp::Subscription<uav_msgs::msg::UavTelemetry>::SharedPtr    sub_telemetry_;

    int altitude_to_focus(double alt_m) const
    {
        const double clamped = std::clamp(alt_m, alt_min_m_, alt_max_m_);
        const double t = (clamped - alt_min_m_) / (alt_max_m_ - alt_min_m_);
        const double f = focus_at_alt_min_
                       + t * (focus_at_alt_max_ - focus_at_alt_min_);
        return std::clamp(static_cast<int>(std::lround(f)), 0, 1000);
    }

    void apply_focus(double alt_m)
    {
        const int focus_val = altitude_to_focus(alt_m);
        const bool ok = v4l2_set_focus(subdev_, focus_val);
        if (ok) {
            RCLCPP_DEBUG(get_logger(),
                "Focus updated -> alt=%.2fm focus_absolute=%d",
                alt_m, focus_val);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                "v4l2_set_focus failed on %s (focus=%d) - check subdev path",
                subdev_.c_str(), focus_val);
        }
    }

    void on_telemetry(const uav_msgs::msg::UavTelemetry::SharedPtr msg)
    {
        const double alt = msg->altitude_baro;
        if (std::isnan(alt) || alt < 0.0) return;

        std::lock_guard<std::mutex> lk(focus_mutex_);
        if (std::abs(alt - last_focus_alt_m_) < focus_deadband_m_) return;
        last_focus_alt_m_ = alt;
        apply_focus(alt);
    }

    static GstFlowReturn on_new_sample_static(GstAppSink * /*sink*/, gpointer data)
    {
        return static_cast<CameraStreamerFocus *>(data)->on_new_sample();
    }

    GstFlowReturn on_new_sample()
    {
        GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
        if (!sample) return GST_FLOW_ERROR;

        GstBuffer * buf = gst_sample_get_buffer(sample);
        GstMapInfo  info{};

        if (gst_buffer_map(buf, &info, GST_MAP_READ)) {
            sensor_msgs::msg::CompressedImage out;
            out.header.stamp    = now();
            out.header.frame_id = kFrameId;
            out.format          = "jpeg";
            out.data.assign(info.data, info.data + info.size);
            pub_->publish(out);
            gst_buffer_unmap(buf, &info);
        }

        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    void run_bus_loop()
    {
        GstBus * bus = gst_element_get_bus(pipeline_);
        while (bus_running_) {
            GstMessage * msg = gst_bus_timed_pop_filtered(
                bus, 200 * GST_MSECOND,
                static_cast<GstMessageType>(
                    GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_EOS));
            if (!msg) continue;
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR: {
                    GError * e{}; gchar * d{};
                    gst_message_parse_error(msg, &e, &d);
                    RCLCPP_ERROR(get_logger(),
                        "GStreamer error: %s (%s)", e->message, d);
                    g_error_free(e); g_free(d);
                    break;
                }
                case GST_MESSAGE_WARNING: {
                    GError * e{}; gchar * d{};
                    gst_message_parse_warning(msg, &e, &d);
                    RCLCPP_WARN(get_logger(),
                        "GStreamer warning: %s (%s)", e->message, d);
                    g_error_free(e); g_free(d);
                    break;
                }
                case GST_MESSAGE_EOS:
                    RCLCPP_WARN(get_logger(), "GStreamer EOS");
                    break;
                default: break;
            }
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<CameraStreamerFocus>());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("camera_streamer_focus"),
            "Fatal: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
