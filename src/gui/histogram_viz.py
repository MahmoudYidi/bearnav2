#!/usr/bin/env python3

import matplotlib
matplotlib.use('Agg')
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from navigros2.msg import FloatList
import matplotlib.pyplot as plt
import numpy as np

class Histogrammer(Node):
    def __init__(self):
        super().__init__('histogram_viz')
        self.pub = self.create_publisher(Image, "histogram_viz", 0)
        self.br = CvBridge()
        self.subscription = None

        # Initialize the node as lazy
        self.lazy_subscription()
        self.get_logger().warn("Histogrammer initialized")

    def lazy_subscription(self):
        if not self.subscription:
            self.get_logger().warn("Subscribing to histogram")
            self.subscription = self.create_subscription(FloatList, "histogram", self._process, 10)

    def unsubscribe(self):
        if self.subscription:
            self.get_logger().warn("Unsubscribing from histogram")
            self.destroy_subscription(self.subscription)
            self.subscription = None

    def _process(self, msg):
        plt.clf()
        fig = plt.figure()
        ax = plt.axes()
        ax.plot(msg.data)
        fig.canvas.draw()

        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        plt.close()

        img_msg = self.br.cv2_to_imgmsg(img, encoding="rgb8")
        self.pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    histogrammer = Histogrammer()

    try:
        rclpy.spin(histogrammer)
    except KeyboardInterrupt:
        pass
    finally:
        histogrammer.unsubscribe()
        histogrammer.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
