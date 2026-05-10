#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

encoding = ['16UC1', '32FC1']

class DepthToColor(Node):
    def __init__(self):
        super().__init__('get_depth_info')
        self.sub = self.create_subscription(Image, '/camera/depth/image_raw', self.topic, 10)
        self.window_name = 'depth_image'
        self.depth_bridge = CvBridge()

    def topic(self, msg):
        depth_image_orin = self.depth_bridge.imgmsg_to_cv2(msg, encoding[1])
        depth_to_color_image = cv.applyColorMap(cv.convertScaleAbs(depth_image_orin, alpha=0.03), cv.COLORMAP_JET)
        cv.imshow(self.window_name, depth_to_color_image)
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    depth_to_color = DepthToColor()
    rclpy.spin(depth_to_color)
    depth_to_color.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
