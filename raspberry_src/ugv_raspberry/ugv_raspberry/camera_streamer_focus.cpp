// camera_streamer_focus.cpp
// ROS 2 node (C++) – camera stream + altitude-driven dynamic focus
//
// Extends camera_streamer with a /data subscription (UavTelemetry).
// When altitude_baro changes by more than FOCUS_DEADBAND_M the lens
// position is updated via V4L2 ioctls on the camera subdevice.
//
// RPi Camera Module 3 focus mapping
//   focus_absolute 0   → infinity (far)
//   focus_absolute 1000 → macro  (close)
//   Formula: focus = clamp(int(FOCUS_GAIN / max(alt_m, 0.5)), 0, 1000)
//   FOCUS_GAIN ≈ 600 gives ~200 at 3 m and ~240 at 2.5 m.

#include <atomic>
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

// ─── Configuration defaults ───────────────────────────────────────────────────

static constexpr int    CAM_WIDTH        = 640;
static constexpr int    CAM_HEIGHT       = 480;
static constexpr int    CAM_FPS          = 30;
static constexpr int    JPEG_QUALITY     = 50;
static constexpr char   TOPIC[]          = "/ugv_camera/image_raw/compressed";
static constexpr char   FRAME_ID[]       = "camera_frame";
static constexpr int    QOS_DEPTH        = 5;
static constexpr double FOCUS_DEADBAND_M = 0.3;   // min altitude change to re-focus
static constexpr double FOCUS_GAIN       = 600.0;  // tunable per lens

// ─── V4L2 focus helper ────────────────────────────────────────────────────────

static bool v4l2_set_focus(const std::string & dev, int value)
{
    int fd = open(dev.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) return false;

    // Disable continuous autofocus
    v4l2_control ctrl{};
    ctrl.id    = V4L2_CID_FOCUS_AUTO;
    ctrl.value = 0;
    ioctl(fd, VIDIOC_S_CTRL, &ctrl);

    // Set manual focus position
    ctrl.id    = V4L2_CID_FOCUS_ABSOLUTE;
    ctrl.value = value;
    bool ok = (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0);

    close(fd);
    return ok;
}

static int altitude_to_focus(double alt_m)
{
    int val = static_cast<int>(FOCUS_GAIN / std::max(alt_m, 0.5));
    return std::max(0, std::min(1000, val));
}

// ─── Node ─────────────────────────────────────────────────────────────────────

class CameraStreamerFocus : public rclcpp::Node {
public:
    CameraStreamerFocus() : Node("camera_streamer_focus")
    {
        declare_parameter("width",          CAM_WIDTH);
        declare_parameter("height",         CAM_HEIGHT);
        declare_parameter("fps",            CAM_FPS);
        declare_parameter("jpeg_quality",   JPEG_QUALITY);
        declare_parameter("v4l2_subdev",    std::string("/dev/v4l-subdev0"));
        declare_parameter("focus_gain",     FOCUS_GAIN);

        const int    w   = get_parameter("width").as_int();
        const int    h   = get_parameter("height").as_int();
        const int    fps = get_parameter("fps").as_int();
        const int    q   = get_parameter("jpeg_quality").as_int();
        subdev_  = get_parameter("v4l2_subdev").as_string();
        focus_gain_ = get_parameter("focus_gain").as_double();

        // ── GStreamer ─────────────────────────────────────────────────────
        gst_init(nullptr, nullptr);

        std::string pipeline_str =
            "libcamerasrc af-mode=0 name=src ! "   // af-mode=0 = AfModeManual
            "video/x-raw,width=" + std::to_string(w) +
            ",height=" + std::to_string(h) +
            ",framerate=" + std::to_string(fps) + "/1,format=NV12 ! "
            "videoconvert ! "
            "jpegenc quality=" + std::to_string(q) + " ! "
            "appsink name=sink sync=false drop=true max-buffers=1 emit-signals=true";

        RCLCPP_INFO(get_logger(), "Pipeline: %s", pipeline_str.c_str());

        GError * err = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &err);
        if (!pipeline_ || err) {
            std::string msg = err ? err->message : "unknown error";
            if (err) g_error_free(err);
            throw std::runtime_error("GStreamer parse_launch: " + msg);
        }

        sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!sink_) throw std::runtime_error("appsink 'sink' not found");

        g_signal_connect(sink_, "new-sample",
            G_CALLBACK(CameraStreamerFocus::on_new_sample_static), this);

        auto qos = rclcpp::QoS(QOS_DEPTH)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(TOPIC, qos);

        // ── /data subscription (altitude telemetry) ───────────────────────
        sub_telemetry_ = create_subscription<uav_msgs::msg::UavTelemetry>(
            "/data", rclcpp::QoS(5).best_effort(),
            [this](const uav_msgs::msg::UavTelemetry::SharedPtr msg) {
                on_telemetry(msg);
            });

        // Start pipeline
        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING)
                == GST_STATE_CHANGE_FAILURE)
            throw std::runtime_error("Failed to start GStreamer pipeline");

        bus_thread_ = std::thread([this] { run_bus_loop(); });

        RCLCPP_INFO(get_logger(),
            "CameraStreamerFocus: %dx%d @ %d fps  JPEG q=%d  subdev=%s",
            w, h, fps, q, subdev_.c_str());
    }

    ~CameraStreamerFocus()
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
    GstElement *           pipeline_{nullptr};
    GstElement *           sink_{nullptr};
    std::thread            bus_thread_;
    std::atomic<bool>      bus_running_{true};
    std::string            subdev_;
    double                 focus_gain_{FOCUS_GAIN};

    double                 last_focus_alt_m_{-999.0};
    std::mutex             focus_mutex_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
    rclcpp::Subscription<uav_msgs::msg::UavTelemetry>::SharedPtr    sub_telemetry_;

    // ── Telemetry callback → update focus if altitude changed enough ──────
    void on_telemetry(const uav_msgs::msg::UavTelemetry::SharedPtr msg)
    {
        const double alt = msg->altitude_baro;
        if (std::isnan(alt) || alt < 0.0) return;

        std::lock_guard<std::mutex> lk(focus_mutex_);
        if (std::abs(alt - last_focus_alt_m_) < FOCUS_DEADBAND_M) return;
        last_focus_alt_m_ = alt;

        int focus_val = altitude_to_focus(alt);
        bool ok = v4l2_set_focus(subdev_, focus_val);
        if (ok) {
            RCLCPP_DEBUG(get_logger(),
                "Focus updated → alt=%.2f m  focus_absolute=%d", alt, focus_val);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                "v4l2_set_focus failed on %s (focus=%d) – check subdev path",
                subdev_.c_str(), focus_val);
        }
    }

    // ── GStreamer frame callback ───────────────────────────────────────────
    static GstFlowReturn on_new_sample_static(GstAppSink * /*sink*/, gpointer data)
    {
        return static_cast<CameraStreamerFocus *>(data)->on_new_sample();
    }

    GstFlowReturn on_new_sample()
    {
        GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
        if (!sample) return GST_FLOW_ERROR;

        GstBuffer * buf  = gst_sample_get_buffer(sample);
        GstMapInfo  info{};

        if (gst_buffer_map(buf, &info, GST_MAP_READ)) {
            sensor_msgs::msg::CompressedImage out;
            out.header.stamp    = now();
            out.header.frame_id = FRAME_ID;
            out.format          = "jpeg";
            out.data.assign(info.data, info.data + info.size);
            pub_->publish(out);
            gst_buffer_unmap(buf, &info);
        }

        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    // ── GStreamer bus error loop ───────────────────────────────────────────
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
                    RCLCPP_ERROR(get_logger(), "GStreamer error: %s (%s)", e->message, d);
                    g_error_free(e); g_free(d);
                    break;
                }
                case GST_MESSAGE_WARNING: {
                    GError * e{}; gchar * d{};
                    gst_message_parse_warning(msg, &e, &d);
                    RCLCPP_WARN(get_logger(), "GStreamer warning: %s (%s)", e->message, d);
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

// ─── main ─────────────────────────────────────────────────────────────────────

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
