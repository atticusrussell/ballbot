#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
encoding = ['16UC1', '32FC1']
class Depth_To_Color:
	def __init__(self):
		rospy.init_node("get_depth_info", anonymous=False)
		self.sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.topic)
		self.window_name = "depth_image"
		self.depth_bridge = CvBridge()
	
	def topic(self,msg):
		depth_image_orin = self.depth_bridge.imgmsg_to_cv2(msg, encoding[1])
		depth_to_color_image = cv.applyColorMap(cv.convertScaleAbs(depth_image_orin, alpha=0.65), cv.COLORMAP_JET)
		cv.imshow(self.window_name, depth_to_color_image)
		cv.waitKey(1)	

if __name__ == '__main__':
    depth_to_color = Depth_To_Color()
    rospy.spin()
