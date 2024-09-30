#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PreprocessorNode(Node):

    def __init__(self):
        super().__init__('preprocessor')

        # Initialize parameters
        self.declare_parameter('hist_equal', False)

        # Initialize publisher and subscriber
        self.pub = self.create_publisher(Image, 'preprocess/output', 0)
        self.sub = self.create_subscription(Image, 'preprocess/input', self.callback, 1000)

        # Initialize CvBridge
        self.br = CvBridge()

        # Get initial parameter value
        self.hist_equal = self.get_parameter('hist_equal').get_parameter_value().bool_value

        # Set up a callback for parameter changes
        self.add_on_set_parameters_callback(self.config_cb)

    def callback(self, msg):
        img = self.br.imgmsg_to_cv2(msg)

        # Apply histogram equalization if the parameter is set to True
        if self.hist_equal:
            img = cv2.equalizeHist(img)

        msg = self.br.cv2_to_imgmsg(img)
        self.pub.publish(msg)

    def config_cb(self, params):
        for param in params:
            if param.name == 'hist_equal':
                self.hist_equal = param.value

        return rclpy.parameter.ParameterValue(bool_value=True)

def main(args=None):
    rclpy.init(args=args)
    node = PreprocessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info("Shutting down Pre-process node")
        pass
    finally:
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
