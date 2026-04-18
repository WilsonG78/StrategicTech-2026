import rclpy
from rclpy.node import Node
from dualtech_msgs.msg import Detection
from sensor_msgs.msg import Image


class DetectionPublisher(Node):

    def __init__(self):
        super().__init__('detection_publisher')
        self.publisher_ = self.create_publisher(Detection, 'detection', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = Detection()
        msg.object_id = self.counter
        msg.object_type = 'example_object'
        msg.object_image = Image()
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published detection: id={msg.object_id}, type="{msg.object_type}"'
        )
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = DetectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
