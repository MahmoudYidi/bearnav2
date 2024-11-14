import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class GotoStartAdjuster(Node):
    def __init__(self):
        super().__init__('goto_start_adjuster')
        
        # Create a publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # Create a subscriber for odometry
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        
        # Target start position (modify with your real target pose from file)
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'  # Set the frame_id to the correct one for your setup
        
        # Read target pose from position.txt
        self.read_position_from_file('positions.txt')

        self.current_pose = None
        self.error_threshold = 0.01  # Threshold to stop adjustment (in meters)
        self.angular_threshold = 0.005  # Threshold for angular error (in radians)

    def read_position_from_file(self, file_name):
        try:
            with open(file_name, 'r') as file:
                lines = file.readlines()
                for line in lines:
                    if "Position:" in line:
                        # Extract x, y, z, w from the line
                        parts = line.split(',')
                        x = float(parts[0].split('=')[1].strip())
                        y = float(parts[1].split('=')[1].strip())
                        z = float(parts[2].split('=')[1].strip())
                        w = float(parts[3].split('=')[1].strip())

                        # Set the target pose based on extracted values
                        self.target_pose.pose.position.x = x
                        self.target_pose.pose.position.y = y
                        self.target_pose.pose.position.z = z
                        self.target_pose.pose.orientation.w = w

                        self.get_logger().info(f"Target Pose from file: x={x}, y={y}, z={z}, w={w}")

        except Exception as e:
            self.get_logger().error(f"Failed to read position from file {file_name}: {e}")

    def odom_callback(self, msg):
        # Read current position from odometry
        self.current_pose = msg.pose.pose

    def adjust_position(self):
        if not self.current_pose:
            return
        
        # Calculate error between current position and target position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        target_x = self.target_pose.pose.position.x
        target_y = self.target_pose.pose.position.y

        dx = target_x - current_x
        dy = target_y - current_y
        distance_error = math.sqrt(dx**2 + dy**2)

        # Calculate the desired angle
        target_angle = math.atan2(dy, dx)
        current_angle = self.get_yaw_from_quaternion(self.current_pose.orientation)
        angle_error = target_angle - current_angle

        # Normalize angle to [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Create Twist message to send cmd_vel
        cmd_vel = Twist()

        # If distance error is large, move forward
        if distance_error > self.error_threshold:
            cmd_vel.linear.x = 0.1  # Move forward with a speed of 0.2 m/s

        # If angular error is large, rotate the robot
        if abs(angle_error) > self.angular_threshold:
            cmd_vel.angular.z = 0.2 * angle_error  # Rotate with speed proportional to error

        # Stop robot when the error is small enough
        if distance_error < self.error_threshold and abs(angle_error) < self.angular_threshold:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.get_logger().info('Robot is at the start position.')

        # Publish cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)

    def get_yaw_from_quaternion(self, quaternion):
        """ Converts quaternion to yaw (rotation around z-axis) """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = GotoStartAdjuster()

    # Spin the node to keep it running
    while rclpy.ok():
        rclpy.spin_once(node)
        node.adjust_position()  # Continuously adjust the robot's position

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
