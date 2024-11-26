#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from bearnav2.msg import Alignment
from bearnav2.srv import SetClockGain
from rclpy.qos import qos_profile_sensor_data 
from rclpy.qos import QoSProfile

from navigator import Navigator


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator')

        self.navigator = Navigator()
        qos = QoSProfile(depth=10)
        

        # Publishers and Subscribers
        self.declare_parameter("cmd_vel_topic", "")
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.pub = self.create_publisher(Twist, cmd_vel_topic, qos)
        self.sub_vel = self.create_subscription(Twist, "map_vel", self.callback_vel, qos) #1000
        self.sub_corr = self.create_subscription(Alignment, "correction_cmd", self.callback_corr, qos) #1000

        # Service Clients
        self.gain_srv = self.create_client(SetClockGain, 'set_clock_gain')
        while not self.gain_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_clock_gain service...')

        # Parameters
        self.declare_parameter('velocity_gain', 1.0)
        self.declare_parameter('turn_gain', 0.0005)
        self.declare_parameter('use_uncertainty', True)

        self.add_on_set_parameters_callback(self.callback_reconfigure)

    def callback_vel(self, msg):
        driven = self.navigator.process(msg)
        self.pub.publish(driven)

    def callback_corr(self, msg):
        self.navigator.correction(msg)

    def callback_reconfigure(self, params):
        self.navigator.update_parameters(params)

        velocity_gain = self.get_parameter('velocity_gain').value
        try:
            req = SetClockGain.Request() 
            req.gain = 1.0 / velocity_gain  
            future = self.gain_srv.call_async(req)  # Use async call
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f"Clock gain set to: {req.gain}")
            else:
                self.get_logger().warn("Service call failed")
        except Exception as e:
            self.get_logger().warn(f"Unable to set clock gain: {str(e)}")

        return params


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info("NavigatorNode interrupted by user.")
        pass
    finally:
        pass
        #rclpy.shutdown()

if __name__ == "__main__":
    main()
