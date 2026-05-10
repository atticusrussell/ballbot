#!/usr/bin/env python
# coding: utf-8

import cv2 as cv
from Calibration import Arm_Calibration

if __name__ == '__main__':
    calibration = Arm_Calibration()
    capture = cv.VideoCapture(0)
    dp = []
    while capture.isOpened():
        _, image = capture.read()
        action = cv.waitKey(10) & 0xff
        # print action
        if action == 49:  # 模拟进入标定后的效果
            print("49")
            # 显示方框检测效果图
            image, (HSV_min, HSV_max) = calibration.get_hsv(image)
        if action == 32:  # 模拟进入标定后的效果
            print("32")
            # 显示方框检测效果图
            dp, image = calibration.calibration_map(image,threshold_num=130)
        if action == 27:  # 模拟标定后的效果
            print("27")
            # 返回仿射变换后的效果图
            image = calibration.Perspective_transform(dp, image)
        cv.imshow("image", image)
    cv.destroyAllWindows()
