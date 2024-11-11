#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from navigros2.srv import SetDist  
from distance import Distance  
from rclpy.qos import qos_profile_sensor_data

class DistanceNode(Node):
    def __init__(self):
        super().__init__('distance_node')

        # Declare parameters
        self.declare_parameter('use_twist', False)
        self.declare_parameter('cmd_vel_topic', '')
        self.declare_parameter('odom_topic', '')

        # Get parameter values
        use_twist = self.get_parameter('use_twist').get_parameter_value().bool_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.d = Distance(use_twist)

        # Create publisher
        self.pub = self.create_publisher(Float64, 'distance', 1)

        # Create subscribers
        self.create_subscription(Odometry, odom_topic, self.callback_odom,qos_profile=qos_profile_sensor_data )
        self.create_subscription(Twist, cmd_vel_topic, self.callback_twist, qos_profile=qos_profile_sensor_data)

        # Create service
        self.srv = self.create_service(SetDist, 'set_dist', self.handle_set_dist)

    def callback_twist(self, msg):
        driven, use = self.d.processT(msg, self.get_clock().now().seconds_nanoseconds()[0])
        if use:
            self.pub.publish(Float64(data=driven))

    def callback_odom(self, msg):
        driven, use = self.d.processO(msg)
        if use:
            self.pub.publish(Float64(data=driven))

    def handle_set_dist(self, request, response):
        driven = self.d.set(request)
        self.get_logger().info(f"Distance set to {driven}")
        self.pub.publish(Float64(data=driven))
        #response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info("Shutting down Distance node")
        pass
    finally:
        node.destroy_node()
        #rclpy.shutdown()


if __name__ == '__main__':
    main()
