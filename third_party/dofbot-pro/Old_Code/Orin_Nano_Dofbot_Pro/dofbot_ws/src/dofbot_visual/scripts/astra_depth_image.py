#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

encoding = ['16UC1', '32FC1']
def topic(msg):
    if not isinstance(msg, Image):
        return
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, encoding[1])
    # 规范输入图像大小
    # Standardize the input image size
    frame = cv.resize(frame, (640, 480))
    # h, w = frame.shape[:2]
    # for row in range(h):
    #     for col in range(w):
    #         print ("x: {}，y:{}，z: {}".format(row, col, frame[row, col] / 1000.0))
    cv.imshow("depth_image", frame)
    cv.waitKey(10)


if __name__ == '__main__':
    rospy.init_node("astra_depth_image_py", anonymous=False)
    sub = rospy.Subscriber("/camera/depth/image_raw", Image, topic)
    rate = rospy.Rate(2)
    rospy.spin()
