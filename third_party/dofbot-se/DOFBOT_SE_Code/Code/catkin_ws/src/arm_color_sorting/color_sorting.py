#!/usr/bin/env python
# coding: utf-8
import random
import Arm_Lib
import threading
import cv2 as cv
from time import sleep
from sorting_move import sorting_move


class color_sorting:
    def __init__(self):
        '''
        设置初始化参数
        '''
        self.image = None
        # 初始化计数器
        self.num = 0
        # 初始化运动状态
        self.status = 'waiting'
        # 创建抓取实例
        self.sorting_move = sorting_move()
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()

    def get_Sqaure(self, color_name, hsv_lu):
        '''
        颜色识别
        '''
        (lowerb, upperb) = hsv_lu
        # 复制原始图像,避免处理过程中干扰
        img = self.image.copy()
        # mask = self.image.copy()
        mask = img[230:450, 220:420]
        # cv.imshow("mask", mask)
        # 将图像转换为HSV。
        HSV_img = cv.cvtColor(mask, cv.COLOR_BGR2HSV)
        # 筛选出位于两个数组之间的元素。
        img = cv.inRange(HSV_img, lowerb, upperb)
        # 设置非掩码检测部分全为黑色
        mask[img == 0] = [0, 0, 0]
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # 将图像转为灰度图
        dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标) python2和python3在此处略有不同
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(contours):
            # boundingRect函数计算边框值，x，y是坐标值，w，h是矩形的宽和高
            x, y, w, h = cv.boundingRect(cnt)
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            # ⾯积范围
            if area > 1000:
                # 中心坐标
                x_w_ = float(x + w / 2)
                y_h_ = float(y + h / 2)
                # 在img图像画出矩形，(x, y), (x + w, y + h)是矩形坐标，(0, 255, 0)设置通道颜色，2是设置线条粗度
                # cv.rectangle(self.image, (x , y ), (x + w , y + h ), (0, 255, 0), 2)
                cv.rectangle(self.image, (x + 220, y + 230), (x + w + 220, y + h + 240), (0, 255, 0), 2)
                # 绘制矩形中心
                # cv.circle(self.image, (int(x_w_ ), int(y_h_ )), 5, (0, 0, 255), -1)
                cv.circle(self.image, (int(x_w_ + 220), int(y_h_ + 230)), 5, (0, 0, 255), -1)
                # # 在图片中绘制结果
                # cv.putText(self.image, color_name, (int(x -15), int(y -15)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                cv.putText(self.image, color_name, (int(x + 202), int(y + 215)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                return (x_w_, y_h_)

    def Sorting_grap(self, img, color_hsv):
        # 规范输入图像大小
        self.image = cv.resize(img, (640, 480))
        # 设置随机颜色
        # color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
        # 画矩形框 color[random.randint(0, 254)]
        cv.rectangle(self.image, (215, 230), (425, 450),(105,105,105), 2)
        # 获取识别的结果
        msg = {}
        # 遍历颜色通道,获取能够识别的结果
        for key, value in color_hsv.items():
            point = self.get_Sqaure(key, value)
            if point != None: msg["name"] = key
        if len(msg) == 1:
            self.num += 1
            # 每当连续识别20次并且运动状态为waiting的情况下,执行抓取任务
            if self.num % 10 == 0 and self.status == 'waiting':
                self.status = "Runing"
                self.arm.Arm_Buzzer_On(1)
                sleep(0.5)
                # 开启抓取线程
                threading.Thread(target=self.sorting_move.sorting_run, args=(msg['name'],)).start()
                self.num = 0
                # 抓取完毕
                self.status = 'waiting'
        else:
            self.num = 0
            self.status = 'waiting'
        return self.image

