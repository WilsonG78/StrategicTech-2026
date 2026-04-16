#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import curses


class ControlNode(Node):
    def __init__(self):
        super().__init__("wsad_node")
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

        self.velocity_speed = 0.5
        self.velocity_rotate = 0.8

        self.is_running = False

    def send_command(self, char, stdscr):
        msg = Twist()

        if char == ord('w') or char == ord('W'):
            msg.linear.x = self.velocity_speed
            stdscr.addstr(8, 0, "Status: FORWARD      ")
            self.is_running = True
            self.publisher_.publish(msg)

        elif char == ord('s') or char == ord('S'):
            msg.linear.x = -self.velocity_speed
            stdscr.addstr(8, 0, "Status: BACKWARD     ")
            self.is_running = True
            self.publisher_.publish(msg)

        elif char == ord('a') or char == ord('A'):
            msg.angular.z = self.velocity_rotate
            stdscr.addstr(8, 0, "Status: TURN LEFT    ")
            self.is_running = True
            self.publisher_.publish(msg)

        elif char == ord('d') or char == ord('D'):
            msg.angular.z = -self.velocity_rotate
            stdscr.addstr(8, 0, "Status: TURN RIGHT   ")
            self.is_running = True
            self.publisher_.publish(msg)

        elif char == ord('=') or char == ord('+'):
            self.velocity_speed += 0.1
            if self.velocity_speed > 1.0:
                self.velocity_speed = 1.0

        elif char == ord('-'):
            self.velocity_speed -= 0.1
            if self.velocity_speed < 0.1:
                self.velocity_speed = 0.1

        elif char == -1:
            if self.is_running:
                stdscr.addstr(8, 0, "Status: STOP         ")
                self.publisher_.publish(msg)
                self.is_running = False

        stdscr.addstr(9, 0, f"Forward speed: {self.velocity_speed:.1f}  ")


def keyLogger(stdscr):
    rclpy.init()
    node = ControlNode()

    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.timeout(100)

    stdscr.addstr("Node Control running\n")
    stdscr.addstr("-----------------------------------\n")
    stdscr.addstr("[ W ] - Forward\n")
    stdscr.addstr("[ S ] - Backward\n")
    stdscr.addstr("[ A ] - Turn left\n")
    stdscr.addstr("[ D ] - Turn right\n")
    stdscr.addstr("[ = ] - Increase speed\n")
    stdscr.addstr("[ - ] - Decrease speed\n")
    stdscr.addstr("[ Q ] - Quit\n\n")

    stdscr.addstr(9, 0, f"Forward speed: {node.velocity_speed:.1f}  ")

    while True:
        char = stdscr.getch()

        if char == ord('q') or char == ord('Q'):
            break

        node.send_command(char, stdscr)
        rclpy.spin_once(node, timeout_sec=0)

    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    try:
        curses.wrapper(keyLogger)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
