import rclpy
from rclpy.node import Node
from dualtech_msgs.msg import Detection
from sensor_msgs.msg import Image


class DetectionSubscriber(Node):

    def __init__(self):
        super().__init__('detection_subscriber')
        self.subscription = self.create_subscription(
            Detection, 'detection', self.listener_callback, 10
        )
        self.image_pub = self.create_publisher(Image, 'detection_image', 10)

    def listener_callback(self, msg):
        self.image_pub.publish(msg.object_image)
        self.get_logger().info(
            f'Republished image from detection id={msg.object_id}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
