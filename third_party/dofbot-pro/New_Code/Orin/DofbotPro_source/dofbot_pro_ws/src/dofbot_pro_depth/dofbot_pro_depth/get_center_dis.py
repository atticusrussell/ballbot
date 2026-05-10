#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

encoding = ['16UC1', '32FC1']

class GetDepthInfo(Node):
    def __init__(self):
        super().__init__('get_depth_info')
        self.sub = self.create_subscription(Image, '/camera/depth/image_raw', self.topic_callback, 10)
        self.window_name = 'depth_image'
        self.y = 240
        self.x = 320
        self.depth_bridge = CvBridge()
    
    def click_callback(self, event, x, y, flags, params):
        if event == 1:
            self.x = x
            self.y = y
            self.get_logger().info(f'Clicked coordinates: x={self.x}, y={self.y}')

    def topic_callback(self, msg):
        depth_image = self.depth_bridge.imgmsg_to_cv2(msg, encoding[1])
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        dist = depth_image_info[self.y, self.x]
        depth_image_colored = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
        self.get_logger().info(f'dist: {dist}')
        dist = round(dist, 3)
        dist_text = f'dist: {dist} mm'
        cv.putText(depth_image_colored, dist_text, (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv.setMouseCallback(self.window_name, self.click_callback)
        cv.circle(depth_image_colored, (self.x, self.y), 1, (0, 0, 0), 10)
        cv.imshow(self.window_name, depth_image_colored)
        cv.waitKey(1)    

def main(args=None):
    rclpy.init(args=args)
    get_depth_info = GetDepthInfo()
    rclpy.spin(get_depth_info)
    get_depth_info.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
