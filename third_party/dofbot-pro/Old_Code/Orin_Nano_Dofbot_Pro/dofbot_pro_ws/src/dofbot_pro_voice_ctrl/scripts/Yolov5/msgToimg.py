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
        self.yolov5_img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像
        self.pub_img = rospy.Publisher("yoloDetect/image", Image, queue_size=1)
        #self.image_sub = rospy.Subscriber("Detect/image_msg", Image_Msg, self.yolov5_image_sub_callback)
        self.img_flip = rospy.get_param("~img_flip", False)
        self.image = Image_Msg()
        exit_code = os.system('rosservice call /camera/set_color_exposure  50')
        
    def yolov5_image_sub_callback(self,data):
        if not isinstance(data, Image_Msg): return
        # 将自定义图像消息转化为图像
        # Convert custom image messages to images
        yolov5_image = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8,
                       buffer=base64.b64decode(data.data))
        self.yolov5_img[:, :, 0], self.yolov5_img[:, :, 1], self.yolov5_img[:, :, 2] = yolov5_image[:, :, 2], yolov5_image[:, :, 1], yolov5_image[:, :, 0]
        self.yolov5_img = cv2.cvtColor(self.yolov5_img, cv2.COLOR_BGR2RGB)
        # 规范输入图像大小
        # Standardize the input image size
        self.img = cv2.resize(self.yolov5_img, (640, 480))
        if self.img_flip == True: self.yolov5_img = cv2.flip(self.yolov5_img, 1)
        # opencv mat ->  ros msg
        msg = self.bridge.cv2_to_imgmsg(self.yolov5_img, "bgr8")
        self.pub_img.publish(msg)
    def image_sub_callback(self, data):
        
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        size = self.img.shape
        #image = Image_Msg()
        self.image.height = size[0] # 480
        self.image.width = size[1] # 640
        self.image.channels = size[2] # 3
        self.image.data = data.data # image_data
        #print(self.image)
            # image.data = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8").data
        self.image_pub.publish(self.image)
        #cv2.imshow("window000", self.img)
        #cv2.waitKey(1)
        ''' callback of image_sub '''
        '''try:
           
            #self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            size = self.img.shape
            image = Image_Msg()
            image.height = size[0] # 480
            image.width = size[1] # 640
            image.channels = size[2] # 3
            image.data = data.data # image_data
            # image.data = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8").data
            self.image_pub.publish(image)
            cv2.imshow("window", self.img)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e) '''
if __name__ == '__main__':
	rospy.init_node('image_listenner', anonymous=True)
	image_listenning = image_listenner()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()