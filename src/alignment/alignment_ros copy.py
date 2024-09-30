#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from navigros2.msg import Alignment, FloatList
import alignment 
import numpy as np

class AlignmentNode(Node):
    def __init__(self):
        super().__init__('alignment')
        self.align = alignment.Alignment()
        self.br = CvBridge()
        self.imgABuf = None

        # Publishers
        self.pub = self.create_publisher(Alignment, 'alignment/output', 0)
        self.pub_hist = self.create_publisher(FloatList, 'histogram', 0)

        # Subscribers
        self.create_subscription(Image, 'alignment/inputCurrent', self.callbackA, 1)
        self.create_subscription(Image, 'alignment/inputMap', self.callbackB, 1)

        # Dynamic Reconfigure (ROS2 parameters instead)
        self.declare_parameter('feature_type', '')
        self.feature_type = self.get_parameter('feature_type').get_parameter_value().string_value
        self.update_method()

        # Parameter change callback
        self.add_on_set_parameters_callback(self.config_cb)

    def callbackA(self, msg):
        #self.get_logger().info('got_map')
        self.imgABuf = self.br.imgmsg_to_cv2(msg)

    def callbackB(self, msg):
        if self.imgABuf is None:
            self.get_logger().warn("Aligner still awaiting map image!")
            return

        imgB = self.br.imgmsg_to_cv2(msg)
        self.get_logger().warn(f"Using alignment method: {self.align.method}") 
        alignment, uncertainty, hist = self.align.process(self.imgABuf, imgB)
        m = Alignment()
        m.alignment = alignment
        m.uncertainty = float(uncertainty)

        #m.uncertainty = uncertainty
        self.pub.publish(m)

        hm = FloatList()
        # Flatten the hist array if it's a NumPy array or nested list
        if isinstance(hist, np.ndarray):
            hist = hist.flatten().tolist()  # Convert NumPy array to flat list
            self.get_logger().info(f"hist content: {hist}")
        else:
            hist = [float(h) for sublist in hist for h in sublist]  # Flatten list of lists
        self.get_logger().info(f"hist content: {hist}")
        #hist = [float(h) for h in hist]  # Ensures all elements are floats
        hm.data = hist
        self.pub_hist.publish(hm)

    def config_cb(self, params):
        if 'feature_type' in params:
            self.feature_type = params['feature_type'].value
            self.update_method()
        return rclpy.parameter.ParameterValue()

    def update_method(self):
        self.align.method = self.feature_type
        self.get_logger().info(f"Alignment method set to: {self.feature_type}")


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        #rclpy.shutdown()
if __name__ == "__main__":
    main()
