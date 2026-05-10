# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
import numpy as np


class snake_target:
    def __init__(self):
        '''
        初始化一些参数
        '''
        self.image = None
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def Image_Processing(self, img):
        '''
        形态学变换去出细小的干扰因素
        :param img: 输入初始图像
        :return: 检测的轮廓点集(坐标)
        '''
        # 将图像转为灰度图
        gray_img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        dst_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标) python2和python3在此处略有不同
        # cv.imshow("th", binary)
        # _, contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) #python2
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # python3
        return contours

    def get_area(self, hsv_name, hsv_range):
        (lowerb, upperb) = hsv_range
        # 复制原始图像,避免处理过程中干扰
        color_mask = self.image.copy()
        # 将图像转换为HSV。
        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        # 筛选出位于两个数组之间的元素。
        color = cv.inRange(hsv_img, lowerb, upperb)
        # 设置非掩码检测部分全为黑色
        color_mask[color == 0] = [0, 0, 0]
        # cv.imshow("mask", color_mask)
        # 检测轮廓点集
        contours = self.Image_Processing(color_mask)
        # 采用多边形逼近的方法绘制轮廓
        for i, cnt in enumerate(contours):
            # 计算多边形的矩
            mm = cv.moments(cnt)
            if mm['m00'] == 0:
                continue
            cx = mm['m10'] / mm['m00']
            cy = mm['m01'] / mm['m00']
            # 获取多边形的中心
            (x, y) = (np.int(cx), np.int(cy))
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            # ⾯积⼤于800
            if area > 800:
                # 绘制中⼼
                cv.circle(self.image, (x, y), 5, (0, 0, 255), -1)
                # 计算最小矩形区域
                rect = cv.minAreaRect(cnt)
                # 获取盒⼦顶点
                box = cv.boxPoints(rect)
                # 转成long类型
                box = np.int0(box)
                # 绘制最小矩形
                cv.drawContours(self.image, [box], 0, (255, 0, 0), 2)
                # 在图片中绘制结果
                cv.putText(self.image, hsv_name, (int(box[1][0] - 15), int(box[1][1]) - 15),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                return area

    def target_run(self, img, color_hsv):
        '''
        颜色跟随控制函数
        '''
        # 规范输入图像大小
        self.image = cv.resize(img, (640, 480), )
        # 遍历颜色通道,获取能够识别的结果
        msg = {}
        for key, value in color_hsv.items():
            area = self.get_area(key, value)
            if area != None: msg[key] = area
        return self.image, msg
