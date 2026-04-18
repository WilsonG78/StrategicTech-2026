// camera_streamer.cpp
// Węzeł ROS 2 (C++) – stream z Camera Module 3 przez libcamera + GStreamer
//
// Priorytet: MINIMALNE OPÓŹNIENIE
// Pipeline: libcamerasrc → jpegenc (q=40–60) → appsink → /camera/image/compressed
//
// Na laptopie podgląd: ros2 run image_transport republish compressed raw --ros-args
//   -r in/compressed:=/camera/image/compressed
// Lub rqt_image_view bezpośrednio.

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// ─── Konfiguracja ─────────────────────────────────────────────────────────────

static constexpr int    CAM_WIDTH     = 640;
static constexpr int    CAM_HEIGHT    = 480;
static constexpr int    CAM_FPS       = 30;
static constexpr int    JPEG_QUALITY  = 50;   // 30–60 = dobry balans latencja/jakość
static constexpr char   TOPIC[]       = "/camera/image/compressed";
static constexpr char   FRAME_ID[]    = "camera_frame";
static constexpr int    QOS_DEPTH     = 5;    // mały bufor = świeże klatki


// ─── Węzeł ────────────────────────────────────────────────────────────────────

class CameraStreamer : public rclcpp::Node {
public:
    CameraStreamer() : Node("camera_streamer") {

        // Parametry nadpisywalne z CLI / launch
        declare_parameter("width",        CAM_WIDTH);
        declare_parameter("height",       CAM_HEIGHT);
        declare_parameter("fps",          CAM_FPS);
        declare_parameter("jpeg_quality", JPEG_QUALITY);

        const int w  = get_parameter("width").as_int();
        const int h  = get_parameter("height").as_int();
        const int fps = get_parameter("fps").as_int();
        const int q  = get_parameter("jpeg_quality").as_int();

        // ── GStreamer init ────────────────────────────────────────────────
        gst_init(nullptr, nullptr);

        // Pipeline zoptymalizowany pod minimalne opóźnienie:
        //  • sync=false na appsink  → nie czeka na zegar
        //  • drop=true              → porzuca stare klatki gdy nie nadążamy
        //  • max-buffers=1          → zawsze najświeższa klatka
        //  • NV12 → videoconvert → jpegenc  (szybszy niż MJPEG z kamery)
        //  • Dla jeszcze mniejszego opóźnienia: zmień jpegenc na avenc_mjpeg
        std::string pipeline_str =
            "libcamerasrc ! "
            "video/x-raw,width=" + std::to_string(w) + ",height=" + std::to_string(h) +
            ",framerate=" + std::to_string(fps) + "/1,format=NV12 ! "
            "videoflip method=rotate-180 ! "
            "videoconvert ! "
            "jpegenc quality=" + std::to_string(q) + " ! "
            "appsink name=sink sync=false drop=true max-buffers=1 emit-signals=true";

        RCLCPP_INFO(get_logger(), "Pipeline: %s", pipeline_str.c_str());

        GError * err = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &err);
        if (!pipeline_ || err) {
            std::string msg = err ? err->message : "nieznany błąd";
            if (err) g_error_free(err);
            throw std::runtime_error("GStreamer parse_launch: " + msg);
        }

        // ── appsink: callback na nową klatkę ─────────────────────────────
        sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!sink_) throw std::runtime_error("Nie znaleziono appsink 'sink'");

        g_signal_connect(sink_, "new-sample",
            G_CALLBACK(CameraStreamer::on_new_sample_static), this);

        // ── Publisher ─────────────────────────────────────────────────────
        auto qos = rclcpp::QoS(QOS_DEPTH)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);  // UDP-like, bez retransmisji
        pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(TOPIC, qos);

        // ── Start pipeline ────────────────────────────────────────────────
        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE)
            throw std::runtime_error("Nie można uruchomić pipeline GStreamer");

        // Monit o błędach z magistrali GStreamer w osobnym wątku
        bus_thread_ = std::thread([this]{ run_bus_loop(); });

        RCLCPP_INFO(get_logger(),
            "CameraStreamer: %dx%d @ %d fps  JPEG quality=%d  topic=%s",
            w, h, fps, q, TOPIC);
    }

    ~CameraStreamer() {
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

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;

    // ── Callback GStreamer (wywoływany z wątku GStreamera) ────────────────
    static GstFlowReturn on_new_sample_static(GstAppSink * /*sink*/, gpointer data) {
        return static_cast<CameraStreamer *>(data)->on_new_sample();
    }

    GstFlowReturn on_new_sample() {
        GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
        if (!sample) return GST_FLOW_ERROR;

        GstBuffer * buf  = gst_sample_get_buffer(sample);
        GstMapInfo  info{};

        if (gst_buffer_map(buf, &info, GST_MAP_READ)) {
            auto msg        = sensor_msgs::msg::CompressedImage();
            msg.header.stamp    = now();
            msg.header.frame_id = FRAME_ID;
            msg.format          = "jpeg";
            msg.data.assign(info.data, info.data + info.size);
            pub_->publish(msg);
            gst_buffer_unmap(buf, &info);
        }

        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    // ── Magistrala GStreamer – obsługa błędów ─────────────────────────────
    void run_bus_loop() {
        GstBus * bus = gst_element_get_bus(pipeline_);
        while (bus_running_) {
            GstMessage * msg = gst_bus_timed_pop_filtered(
                bus, 200 * GST_MSECOND,
                static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_EOS)
            );
            if (!msg) continue;

            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR: {
                    GError * err{};
                    gchar  * dbg{};
                    gst_message_parse_error(msg, &err, &dbg);
                    RCLCPP_ERROR(get_logger(), "GStreamer error: %s (%s)", err->message, dbg);
                    g_error_free(err); g_free(dbg);
                    break;
                }
                case GST_MESSAGE_WARNING: {
                    GError * err{};
                    gchar  * dbg{};
                    gst_message_parse_warning(msg, &err, &dbg);
                    RCLCPP_WARN(get_logger(), "GStreamer warning: %s (%s)", err->message, dbg);
                    g_error_free(err); g_free(dbg);
                    break;
                }
                case GST_MESSAGE_EOS:
                    RCLCPP_WARN(get_logger(), "GStreamer EOS – koniec strumienia");
                    break;
                default: break;
            }
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
    }
};


// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<CameraStreamer>());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("camera_streamer"), "Błąd: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
