import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_udp_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)
  
        self.cap = cv2.VideoCapture('tcp://10.158.100.219:8888')
        
        if not self.cap.isOpened():
            self.get_logger().error("Nie można połączyć się z kamerą!")
            return

        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Węzeł kamery uruchomiony. Rozpoczynam nadawanie do ROS 2 (UDP/DDS)...")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            msg.format = "jpeg"
            
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            success, encoded_image = cv2.imencode('.jpg', frame, encode_param)
            
            if success:
                msg.data = encoded_image.tobytes()
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
        
    camera_publisher.cap.release()
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()