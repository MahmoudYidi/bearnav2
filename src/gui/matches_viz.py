#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from navigros2.msg import FeatureLocations

class FeatureViz(Node):
    def __init__(self):
        super().__init__('feature_viz')

        # Publishers
        self.train_pub = self.create_publisher(Image, 'image_viz/train', 0)
        self.current_pub = self.create_publisher(Image, 'image_viz/current', 0)
        self.both_pub = self.create_publisher(Image, 'image_viz/alignment', 0)

        # Subscribers
        self.create_subscription(Image, 'alignment/inputCurrent', self.cb_img_a, 1)
        self.create_subscription(Image, 'alignment/inputMap', self.cb_img_b, 1)
        self.create_subscription(FeatureLocations, 'alignment/featureLocations', self.cb_fl, 1)

        # CV Bridge
        self.br = CvBridge()

        # Image buffers
        self.img_a = None
        self.img_b = None

        # Timestamp buffers
        self.img_a_ts = None
        self.img_b_ts = None

        # Timeout for images
        self.img_timeout = 0.5  # seconds

        # Draw parameters
        self.match_col = (0, 255, 0)  # Green
        self.nomatch_col = (255, 0, 0)  # Red
        self.box_size = 10

    def is_publishing(self):
        return (self.train_pub.get_subscription_count() > 0 or
                self.current_pub.get_subscription_count() > 0 or
                self.both_pub.get_subscription_count() > 0)

    def cb_img_a(self, msg):
        if self.is_publishing():
            self.img_a = self.br.imgmsg_to_cv2(msg)
            self.img_a_ts = self.get_clock().now()
            self.get_logger().info("Received image A")

    def cb_img_b(self, msg):
        if self.is_publishing():
            self.img_b = self.br.imgmsg_to_cv2(msg)
            self.img_b_ts = self.get_clock().now()
            self.get_logger().info("Received image B")

    def cb_fl(self, msg):
        if not self.is_publishing():
            return

        if self.img_a is None or self.img_b is None:
            self.get_logger().warn("Waiting for images...")
            return

        self.get_logger().info("Received feature locations, processing...")
        current_time = self.get_clock().now()

        # Check if images are too old
        a_diff = (current_time - self.img_a_ts).nanoseconds * 1e-9
        if a_diff > self.img_timeout:
            self.get_logger().warn("Not drawing feats on img A, too old")
            return

        b_diff = (current_time - self.img_b_ts).nanoseconds * 1e-9
        if b_diff > self.img_timeout:
            self.get_logger().warn("Not drawing feats on img B, too old")
            return

        for idx in range(len(msg.xa)):
            xa, ya = msg.xa[idx], msg.ya[idx]
            color = self.match_col if msg.matched[idx] else self.nomatch_col
            cv2.rectangle(self.img_a, (xa - self.box_size, ya - self.box_size),
                          (xa + self.box_size, ya + self.box_size), color, 1)

        # Publish the modified images
        train_img_msg = self.br.cv2_to_imgmsg(self.img_a, encoding="bgr8")
        current_img_msg = self.br.cv2_to_imgmsg(self.img_b, encoding="bgr8")

        self.train_pub.publish(train_img_msg)
        self.current_pub.publish(current_img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FeatureViz()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == "__main__":
    main()
