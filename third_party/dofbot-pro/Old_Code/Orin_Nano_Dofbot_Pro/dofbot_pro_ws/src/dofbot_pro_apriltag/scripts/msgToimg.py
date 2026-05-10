#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import sys
import rospy
import numpy as np
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from dofbot_pro_info.msg import Image_Msg
import base64
class image_listenner:
    def __init__(self): 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.image_sub_callback)
        self.image_pub = rospy.Publisher('/image_data', Image_Msg, queue_size=1)
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像
        self.img_flip = rospy.get_param("~img_flip", False)
        self.image = Image_Msg()
        

    def image_sub_callback(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        size = self.img.shape
        self.image.height = size[0] # 480
        self.image.width = size[1] # 640
        self.image.channels = size[2] # 3
        self.image.data = data.data # image_data
        self.image_pub.publish(self.image)

if __name__ == '__main__':
	rospy.init_node('image_listenner', anonymous=True)
	image_listenning = image_listenner()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()