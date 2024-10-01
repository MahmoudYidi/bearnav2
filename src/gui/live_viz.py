#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class LiveFeatures(Node):
    def __init__(self):
        super().__init__('live_features')

        # CV Bridge
        self.br = CvBridge()

        # Publisher
        self.pub = self.create_publisher(Image, 'live_viz', 0)

        # Parameters
        self.declare_parameter('feature_type', 0)
        self.declare_parameter('camera_topic', '')
        self.feature_type_index = self.get_parameter('feature_type').get_parameter_value().integer_value

        # Subscriber
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.create_subscription(Image, camera_topic, self.img_cb, 10)
        # self.create_subscription(CompressedImage, "/camera_2/image_raw/compressed", self.img_compressed_cb, 10)

        # Feature Detectors
        self.rect_size = 8
        self.rect_col = (255, 0, 0)  # BGR
        self.feature_names = ['SIFT', 'SURF', 'KAZE', 'AKAZE', 'BRISK', 'ORB']
        self.feature_types = [None] * 6  # Initialize with None

        # Initialize feature detectors with try-except
        self.initialize_feature_detectors()

        # Set initial feature detector
        self.set_feature_detector(self.feature_type_index)

        # Timer to check parameter updates
        self.create_timer(1.0, self.update_parameters)

    def initialize_feature_detectors(self):
        try:
            self.feature_types[0] = cv2.SIFT_create()
            self.feature_types[1] = cv2.xfeatures2d.SURF_create()
            self.feature_types[2] = cv2.KAZE_create()
            self.feature_types[3] = cv2.AKAZE_create()
            self.feature_types[4] = cv2.BRISK_create()
            self.feature_types[5] = cv2.ORB_create()
        except Exception as e:
            #self.get_logger().error(f"Error initializing feature detectors: {e}")
            # Fallback to available detectors, excluding those that failed
            self.feature_types[0] = cv2.SIFT_create()
            self.feature_types[2] = cv2.KAZE_create()
            self.feature_types[3] = cv2.AKAZE_create()
            self.feature_types[4] = cv2.BRISK_create()
            self.feature_types[5] = cv2.ORB_create()

    def set_feature_detector(self, index):
        if 0 <= index < len(self.feature_types) and self.feature_types[index] is not None:
            self.feature_detector = self.feature_types[index]
            self.get_logger().info(f"Feature Detector set to: {self.feature_names[index]}")
        else:
            self.get_logger().warn(f"Feature index '{index}' is not valid. Defaulting to ORB.")
            self.feature_detector = self.feature_types[-1]  # Default to ORB

    def update_parameters(self):
        new_feature_type_index = self.get_parameter('feature_type').get_parameter_value().integer_value
        if new_feature_type_index != self.feature_type_index:
            self.feature_type_index = new_feature_type_index
            self.set_feature_detector(new_feature_type_index)

    def is_publishing(self):
        return self.pub.get_subscription_count() > 0

    def img_cb(self, msg):
        if self.is_publishing():
            self.detect_img(msg, compressed=False)

    def img_compressed_cb(self, msg):
        if self.is_publishing():
            self.detect_img(msg, compressed=True)

    def detect_img(self, msg, compressed):
        img = None
        if compressed:
            img = self.br.compressed_imgmsg_to_cv2(msg)
        else:
            img = self.br.imgmsg_to_cv2(msg)

        kps, des = self.feature_detector.detectAndCompute(img, None)

        for kp in kps:
            x = int(kp.pt[0])
            y = int(kp.pt[1])
            cv2.rectangle(img, (x - self.rect_size, y - self.rect_size),
                          (x + self.rect_size, y + self.rect_size), self.rect_col, 1)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_msg = self.br.cv2_to_imgmsg(img, encoding="rgb8")
        self.pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LiveFeatures()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        #rclpy.shutdown()


if __name__ == "__main__":
    main()
