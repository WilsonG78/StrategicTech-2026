import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Device
from gpiozero import Robot

class ControlerNode(Node):
    def __init__(self):
        super().__init__("control_listener")
        self.robot = Robot(left=(27, 17), right=(23, 22))
        self.subscriber_ = self.create_subscription(Twist, "/cmd_vel", self.callback_control, 10)
        
        self.target_linear_x = 0.0
        self.current_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_angular_z = 0.0
        
        self.acceleration_step = 0.05
        self.timer = self.create_timer(0.02, self.control_loop)

    def callback_control(self, msg):
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z

    def control_loop(self):
        is_changed = False

        if self.current_linear_x < self.target_linear_x:
            self.current_linear_x = min(self.target_linear_x, self.current_linear_x + self.acceleration_step)
            is_changed = True
        elif self.current_linear_x > self.target_linear_x:
            self.current_linear_x = max(self.target_linear_x, self.current_linear_x - self.acceleration_step)
            is_changed = True

        if self.current_angular_z < self.target_angular_z:
            self.current_angular_z = min(self.target_angular_z, self.current_angular_z + self.acceleration_step)
            is_changed = True
        elif self.current_angular_z > self.target_angular_z:
            self.current_angular_z = max(self.target_angular_z, self.current_angular_z - self.acceleration_step)
            is_changed = True

        power_x = min(abs(self.current_linear_x), 1.0)
        power_z = min(abs(self.current_angular_z), 1.0)

        if self.current_linear_x > 0.01:
            self.robot.forward(speed=power_x)
        elif self.current_linear_x < -0.01:
            self.robot.backward(speed=power_x)
        elif self.current_angular_z > 0.01:
            self.robot.left(speed=power_z)
        elif self.current_angular_z < -0.01:
            self.robot.right(speed=power_z)
        else:
            self.robot.stop()

        if is_changed:
            self.get_logger().info(
                f"RAMP -> X [Target: {self.target_linear_x:.2f} | Current: {self.current_linear_x:.2f}]  "
                f"Z [Target: {self.target_angular_z:.2f} | Current: {self.current_angular_z:.2f}]"
            )

def main(args=None):
    rclpy.init(args=args)
    node = ControlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()