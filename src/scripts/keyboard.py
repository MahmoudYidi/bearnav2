import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import os
import atexit

class KeyboardDriveNode(Node):
    def __init__(self):
        super().__init__('keyboard_drive_node')
        self.publisher_ = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 0) #Standalone so you can change
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Keyboard Drive Node has started.")
        self.cmd_vel = Twist()
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        atexit.register(self.restore_terminal_settings)
    
    def timer_callback(self):
        key = self.get_key()
        if key:
            if key == 'w':  # Move forward
                self.cmd_vel.linear.x = 1.0
                self.cmd_vel.angular.z = 0.0
            elif key == 's':  # Move backward
                self.cmd_vel.linear.x = -1.0
                self.cmd_vel.angular.z = 0.0
            elif key == 'a':  # Turn left
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 2.5  # Increase this value for faster turning
            elif key == 'd':  # Turn right
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = -2.5  # Increase this value for faster turning
            elif key == 'q':  # Stop
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
            else:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0

            self.publisher_.publish(self.cmd_vel)

    def get_key(self):
        try:
            ch = sys.stdin.read(1)
            if ch:
                return ch
        except IOError:
            return None

    def restore_terminal_settings(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
