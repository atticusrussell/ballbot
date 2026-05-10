#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2 as cv
import message_filters
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class pointcloud:
    def __init__(self):
        rospy.init_node("astra_pcl_py", anonymous=False)
        self.rgb_img = None
        self.depth_img = None
        self.bridge = CvBridge()
        # one callback that deals with depth and rgb at the same time
        im_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        dep_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 10, 0.5)
        self.timeSynchronizer.registerCallback(self.trackObject)
        rospy.spin()

    def trackObject(self, image_data, depth_data):
        if not isinstance(image_data, Image): return
        if not isinstance(depth_data, Image): return
        # convert both images to numpy arrays
        frame = self.bridge.imgmsg_to_cv2(image_data, 'bgr8')
        depthFrame = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough')  # "32FC1")
        cv.imshow("frame", frame)
        cv.imshow("depthFrame", depthFrame)
        cv.waitKey(10)

if __name__ == '__main__':
    pointcloud()

