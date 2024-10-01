#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from navigros2.msg import Alignment, FloatList
from rclpy.parameter import Parameter
import alignment
import numpy as np

class AlignmentNode(Node):
    def __init__(self):
        super().__init__('alignment')
        self.br = CvBridge()
        self.imgABuf = None
        self.align_feature_type = self.declare_parameter('feature_type', 'SIFT').value

        self.aligner = alignment.Alignment()
        self.aligner.method = self.align_feature_type

        self.pub = self.create_publisher(Alignment, 'alignment/output', 0)
        self.pub_hist = self.create_publisher(FloatList, 'histogram', 0)

        self.create_subscription(Image, 'alignment/inputMap', self.callbackA, 1)
        self.create_subscription(Image, 'alignment/inputCurrent', self.callbackB, 1)

        self.get_logger().info("Aligner Ready...")

    def callbackA(self, msg):
        #self.get_logger().info("Received map image")
        self.imgABuf = self.br.imgmsg_to_cv2(msg)

    def callbackB(self, msg):
        if self.imgABuf is None:
            self.get_logger().warn("Aligner still awaiting map image!")
            return

        imgB = self.br.imgmsg_to_cv2(msg)
        alignment_result, uncertainty, hist = self.aligner.process(self.imgABuf, imgB)
        
        m = Alignment()
        m.alignment = alignment_result
        m.uncertainty = uncertainty
        self.pub.publish(m)
        
        #print(hist)
        #hm = FloatList()
        #hm.data = hist
        
        hm = FloatList()
        
        if isinstance(hist, np.ndarray):
            hist = hist.flatten().tolist()  
            #self.get_logger().info(f"hist content: {hist}")
            
        else:
            hist = [float(h) for sublist in hist for h in sublist]  
        
       # print(hist)
        #hist = [float(h) for h in hist]  
        hm.data = hist
        self.pub_hist.publish(hm)


def main(args=None):
    rclpy.init(args=args)
    alignment_node = AlignmentNode()

    try:
        rclpy.spin(alignment_node)
    except KeyboardInterrupt:
        pass
    finally:
        alignment_node.destroy_node()
        #rclpy.shutdown()

if __name__ == "__main__":
    main()
