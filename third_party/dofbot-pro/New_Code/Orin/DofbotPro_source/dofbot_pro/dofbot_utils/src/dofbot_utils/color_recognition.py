#!/usr/bin/env python3
# coding: utf-8
import cv2 as cv

class ColorRecognition:
    def __init__(self):
        self.image = None

    def get_Sqaure(self, color_name, hsv_lu):
        (lowerb, upperb) = hsv_lu
        # Copy the original image to avoid interference during processing
        # 复制原始图像,避免处理过程中干扰
        mask = self.image.copy()
        # Convert image to HSV
        # 将图像转换为HSV
        HSV_img = cv.cvtColor(mask, cv.COLOR_BGR2HSV)
        # filter out elements between two arrays
        # 筛选出位于两个数组之间的元素
        img = cv.inRange(HSV_img, lowerb, upperb)
        # Set the non-mask detection part to be all black
        # 设置非掩码检测部分全为黑色
        mask[img == 0] = [0, 0, 0]
        # 将图像转为灰度图
        dst_img = cv.cvtColor(mask, cv.COLOR_RGB2GRAY)
        # Get structuring elements of different shapes
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # morphological closure
        # 形态学闭操作
        dst_img = cv.morphologyEx(dst_img, cv.MORPH_CLOSE, kernel)
        # Convert image to grayscale
        # Image Binarization Operation
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # Get the set of contour points (coordinates)
        # 获取轮廓点集(坐标)
        find_contours = cv.findContours(
            binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3:
            contours = find_contours[1]
        else:
            contours = find_contours[0]
        for i, cnt in enumerate(contours):
            x, y, w, h = cv.boundingRect(cnt)
            # Calculate the area of ​​the contour
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            if area > 800:
                # Central coordinates
                # 中心坐标
                x_w_ = float(x + w / 2)
                y_h_ = float(y + h / 2)
                cv.rectangle(self.image, (x, y),
                             (x + w, y + h), (0, 255, 0), 2)
                cv.circle(self.image, (int(x_w_), int(y_h_)),
                          5, (0, 0, 255), -1)
                cv.putText(self.image, color_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                return (x_w_, y_h_)

    def getcolor(self, img, color_hsv):
        self.image = cv.resize(img, (640, 480))
        color = ""
        for key, value in color_hsv.items():
            point = self.get_Sqaure(key, value)
            if point != None:
                color = key
                break
        return self.image, color
    
    def get_all_color(self, img, color_hsv):
        # self.image = cv.resize(img, (640, 480))
        self.image = img
        color = ""
        result = []
        for key, value in color_hsv.items():
            point = self.get_Sqaure(key, value)
            if point != None:
                color = key
                result.append(color)
        return self.image, result
