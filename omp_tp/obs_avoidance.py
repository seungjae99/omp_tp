#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

class MyNode(Node):
    def __init__(self):
        super().__init__("csj_image_sub")
        self.image_sub = self.create_subscription(Image, "/image", self.sub_callback, 10)
        self.image_pub = self.create_publisher(Image, "image_proc", 10)
        
    def sub_callback(self, img_msg:Image):
        self.get_logger().info(f"{img_msg.height=} {img_msg.width=}")
        
        img = bridge.imgmsg_to_cv2(img_msg)
        
        img2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img2_msg = bridge.cv2_to_imgmsg(img2)
        
        self.image_pub.publish(img2_msg)
        
def main(args=None):
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()