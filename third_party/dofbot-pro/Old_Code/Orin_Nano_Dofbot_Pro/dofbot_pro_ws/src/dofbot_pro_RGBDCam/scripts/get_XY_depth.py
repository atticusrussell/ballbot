#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

encoding = ['16UC1', '32FC1']

class Get_Depth_Info:
	def __init__(self):
		rospy.init_node("get_depth_info", anonymous=False)
		self.sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.topic)
		self.window_name = "depth_image"
		self.y = 240
		self.x = 320
		self.depth_bridge = CvBridge()
	
	def click_callback(self, event, x, y, flags, params):
		if event == 1:
			self.x = x
			self.y = y
			print(self.x)
			print(self.y)

	def topic(self,msg):
		depth_image = self.depth_bridge.imgmsg_to_cv2(msg, encoding[1])
		frame = cv.resize(depth_image, (640, 480))
		depth_image_info = frame.astype(np.float32)
		dist = depth_image_info[self.y,self.x]
		depth_image_orin = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.65), cv.COLORMAP_JET)
		print("dist: ",dist)
		dist = round(dist,3)
		dist = 'dist: ' + str(dist) + 'mm'
		cv.putText(depth_image_orin, dist,  (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
		cv.setMouseCallback(self.window_name, self.click_callback)
		cv.circle(depth_image_orin,(self.x,self.y),1,(0,0,0),10)
		cv.imshow(self.window_name, depth_image_orin)
		cv.waitKey(1)	

if __name__ == '__main__':
    get_depth_info = Get_Depth_Info()
    rospy.spin()
