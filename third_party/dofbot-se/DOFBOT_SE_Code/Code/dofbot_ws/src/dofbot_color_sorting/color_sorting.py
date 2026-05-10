#!/usr/bin/env python
# coding: utf-8
import random
import Arm_Lib
import threading
import cv2 as cv
from time import sleep


class color_sorting:
    def __init__(self):
        self.image = None
        self.num = 0
        self.status = 'waiting'
        self.arm = Arm_Lib.Arm_Device()
        self.grap_joint = 135
        self.joints = [90, 53, 33, 36, 90, 30]

    def get_Sqaure(self, color_name, hsv_lu):
        (lowerb, upperb) = hsv_lu
        # Set identification area
        # 设置识别区域
        point_Xmin=50   #200
        point_Xmax=600  #440
        point_Ymin=80   #200
        point_Ymax=480  #480
        # draw a rectangle
        # 画矩形框
        # cv.rectangle(self.image, (point_Xmin, point_Ymin), (point_Xmax, point_Ymax),(105,105,105), 2)
        # Copy the original image to avoid interference during processing
        # 复制原始图像,避免处理过程中干扰
        img = self.image.copy()
        mask = img[point_Ymin:point_Ymax, point_Xmin:point_Xmax]
        # mask = self.image.copy()
        # Convert image to HSV
        # 将图像转换为HSV
        HSV_img = cv.cvtColor(mask, cv.COLOR_BGR2HSV)
        # filter out elements between two arrays
        # 筛选出位于两个数组之间的元素
        img = cv.inRange(HSV_img, lowerb, upperb)
        # Set the non-mask detection part to be all black
        # 设置非掩码检测部分全为黑色
        mask[img == 0] = [0, 0, 0]
        # Get structuring elements of different shapes
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # morphological closure
        # 形态学闭操作
        dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # Convert image to grayscale
        # 将图像转为灰度图
        dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)
        # Image Binarization Operation
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # Get the set of contour points (coordinates)
        # 获取轮廓点集(坐标)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3: contours = find_contours[1]
        else: contours = find_contours[0]
        for i, cnt in enumerate(contours):
            x, y, w, h = cv.boundingRect(cnt)
            # Calculate the area of ​​the contour
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            if area > 1000:
                # Central coordinates
                # 中心坐标
                x_w_ = float(x + w / 2)
                y_h_ = float(y + h / 2)
                cv.rectangle(self.image, (x + point_Xmin, y + point_Ymin), (x + w + point_Xmin, y + h + point_Ymin), (0, 255, 0), 2)
                # drawing center
                # 绘制中⼼
                cv.circle(self.image, (int(x_w_ + point_Xmin), int(y_h_ + point_Ymin)), 5, (0, 0, 255), -1)
                cv.putText(self.image, color_name, (int(x + point_Xmin-15), int(y + point_Ymin-15)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
#                 cv.rectangle(self.image, (x , y ), (x + w , y + h ), (0, 255, 0), 2)
#                 cv.circle(self.image, (int(x_w_ ), int(y_h_ )), 5, (0, 0, 255), -1)
#                 cv.putText(self.image, color_name, (int(x -15), int(y -15)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                return (x_w_, y_h_)

    def Sorting_grap(self, img, color_hsv):
        self.image = cv.resize(img, (640, 480))
        msg = {}
        if self.status == 'waiting':
            # Traverse the color channel to obtain recognizable results
            # 遍历颜色通道,获取能够识别的结果
            for key, value in color_hsv.items():
                point = self.get_Sqaure(key, value)
                if point != None: msg["name"] = key
            if len(msg) == 1:
                self.num += 1
                # The grasping task is performed whenever the recognition is performed 10 times continuously and the motion state is waiting
                # 每当连续识别10次并且运动状态为waiting的情况下,执行抓取任务
                if self.num % 10 == 0 and self.status == 'waiting':
                    self.status = "Runing"
                    self.arm.Arm_Buzzer_On(1)
                    sleep(0.5)
                    # Start grab thread
                    # 开启抓取线程
                    threading.Thread(target=self.sorting_run, args=(msg['name'],)).start()
                    self.num = 0
        return self.image

    def sorting_move(self, joints_target):
        '''
        Moving process
        移动过程
        '''
        joints_up = [90, 80, 35, 40, 90, self.grap_joint]
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # Release clamping jaws 松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # Move to object position 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(self.joints, 1000)
        sleep(1)
        # Grasp and clamp the clamping claw进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # rotate 旋转
        self.arm.Arm_serial_servo_write(1, joints_target[0], 500)
        sleep(0.5)
        # Move to corresponding position 移动至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_target, 1000)
        sleep(1.5)
        # Release the object and release the clamping jaws释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # raise  抬起
        joints_up[0] = joints_target[0]
        self.arm.Arm_serial_servo_write6_array(joints_up, 500)
        sleep(0.5)
        # return to center 返回至中心位置
        self.arm.Arm_serial_servo_write(1, 90, 500)
        sleep(0.5)
        # initial position 初始位置
        joints_0 = [90, 130, 0, 0, 90, 0]
        # move to initial position 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
        sleep(1)

    def sorting_run(self, name):
        '''
        Manipulator movement function 机械臂移动函数
        '''
        if name == "red" :
            # print("red")
            # Object placement pose 物体放置位姿
            # joints_target = [115, 20, 80, 40, 90, self.grap_joint]
            joints_target = [117, 19, 66, 56, 90, self.grap_joint]
            # move 移动
            self.sorting_move(joints_target)
            # Grab complete 抓取完毕
            self.status = 'waiting'
        if name == "blue":
            # print("blue")
            # joints_target = [45, 80, 0, 40, 90, self.grap_joint]
            joints_target = [44, 66, 20, 28, 90, self.grap_joint]
            self.sorting_move(joints_target)
            # Grab complete 抓取完毕
            self.status = 'waiting'
        if name == "green" :
            # print("green")
            # joints_target = [137, 80, 0, 40, 90, self.grap_joint]
            joints_target = [136, 66, 20, 29, 90, self.grap_joint]
            self.sorting_move(joints_target)
            # Grab complete 抓取完毕
            self.status = 'waiting'
        if name == "yellow" :
            # print("yellow")
            # joints_target = [65, 20, 80, 40, 90, self.grap_joint]
            joints_target = [65, 22, 64, 56, 90, self.grap_joint]
            self.sorting_move(joints_target)
            # Grab complete 抓取完毕
            self.status = 'waiting'

