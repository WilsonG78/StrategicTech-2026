import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from dualtech_msgs.msg import Detection
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")

        # ── Parameters from launch file ──────────────────────────────────────
        self.declare_parameter("yolo_model",       "best.pt")
        self.declare_parameter("yolo_conf",        0.5)
        self.declare_parameter("frames_to_skip",   3)
        self.declare_parameter("publish_result",   True)

        model_path      = self.get_parameter("yolo_model").get_parameter_value().string_value
        self.conf       = self.get_parameter("yolo_conf").get_parameter_value().double_value
        self.frame_skip = self.get_parameter("frames_to_skip").get_parameter_value().integer_value
        pub_result      = self.get_parameter("publish_result").get_parameter_value().bool_value

        self.get_logger().info(f"Model: {model_path}  conf={self.conf}  skip_frames={self.frame_skip}")

        # ── YOLO ─────────────────────────────────────────────────────────────
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.detection_counter = 0

        # ── QoS — same as camera_streamer.cpp ────────────────────────────────
        qos = QoSProfile(
            depth       = 5,
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history     = HistoryPolicy.KEEP_LAST
        )

        # ── Subscription ─────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            CompressedImage,
            "/camera/image/compressed",
            self.callback,
            qos
        )

        # ── Publisher: annotated image (optional) ────────────────────────────
        self.pub = self.create_publisher(
            CompressedImage, "/detection/image/compressed", qos
        ) if pub_result else None

        # ── Publisher: Detection messages for organizer ──────────────────────
        self.detection_pub = self.create_publisher(Detection, "detection", 10)

        self.frame_num = 0
        self.get_logger().info("DetectorNode ready")

    def callback(self, msg: CompressedImage):
        self.frame_num += 1
        if self.frame_num % self.frame_skip != 0:
            return

        # ── JPEG → BGR ───────────────────────────────────────────────────────
        buf   = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Frame decoding error")
            return

        # ── Detection ────────────────────────────────────────────────────────
        results   = self.model(frame, conf=self.conf, verbose=False)
        annotated = results[0].plot()

        # ── Publish Detection message for each detected object ───────────────
        for box in results[0].boxes:
            class_name = self.model.names[int(box.cls[0])]
            confidence = float(box.conf[0])
            self.get_logger().info(f"  [{class_name}] {confidence:.1%}")

            # Crop the bounding box region
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            crop = frame[y1:y2, x1:x2]

            # Convert BGR crop to sensor_msgs/Image
            ros_image = self.bridge.cv2_to_imgmsg(crop, encoding="bgr8")
            ros_image.header = msg.header

            det_msg = Detection()
            det_msg.object_id    = self.detection_counter
            det_msg.object_type  = class_name
            det_msg.object_image = ros_image
            self.detection_pub.publish(det_msg)
            self.detection_counter += 1

        # ── Publish annotated image ───────────────────────────────────────────
        if self.pub:
            ok, jpg = cv2.imencode(".jpg", annotated,
                                   [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                out        = CompressedImage()
                out.header = msg.header
                out.format = "jpeg"
                out.data   = jpg.tobytes()
                self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
