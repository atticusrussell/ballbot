#!/usr/bin/env python
# coding: utf-8
import rclpy
from rclpy.node import Node
import Arm_Lib
import cv2 as cv
import numpy as np
from time import sleep
from identify_grap import identify_grap
from dofbot_pro_interface.srv import *
import math

rclpy.init()

class identify_GetTarget(Node):
    def __init__(self):
        super().__init__('dofbot_identify')
        self.grap = identify_grap()
        # Robotic arm recognition position adjustment
        # 机械臂识别位置调节
        self.xy = [90, 106]
        # Create a robotic arm instance
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        # Creates a handle to the ROS service to invoke.
        # 创建用于调用的ROS服务的句柄
        self.client = self.create_client(Kinemarics, "dofbot_kinemarics")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def select_color(self, image, color_hsv, color_list):
        '''
        Choose a recognition color
        选择识别颜色
        :param image:输入图像  input image
        :param color_hsv: HSV的范围阈值  Range threshold for HSV
        :param color_list: 颜色序列:['0'：无 '1'：红色 '2'：绿色 '3'：蓝色 '4'：黄色]  Color sequence: ['0': None '1': Red '2': Green '3': Blue '4': Yellow]
        :return: 输出处理后的图像,(颜色,位置)  Output the processed image, (color, position)
        '''
        # canonical input image size
        # 规范输入图像大小
        self.image = cv.resize(image, (640, 480))
        msg = {}
        coo1 = ''
        coo2 = ''
        coo3 = ''
        coo4 = ''
        if len(color_list)==0:return self.image,msg
        if '1' in color_list:
            self.color_name=color_list['1']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: 
                msg[self.color_name] = pos
                coo1 = self.color_name
        if '2' in color_list:
            self.color_name=color_list['2']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: 
                msg[self.color_name] = pos
                coo2 = self.color_name
        if '3' in color_list:
            self.color_name=color_list['3']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: 
                msg[self.color_name] = pos
                coo3 = self.color_name
        if '4' in color_list:
            self.color_name=color_list['4']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: 
                msg[self.color_name] = pos
                coo4 = self.color_name
        return self.image, msg, coo1, coo2, coo3, coo4


    def get_Sqaure(self, color_hsv):
        '''
        Color recognition, get the coordinates of the square
        颜色识别,获得方块的坐标
        '''
        (lowerb, upperb) = color_hsv
        # Copy the original image to avoid interference during processing
        # 复制原始图像,避免处理过程中干扰
        mask = self.image.copy()
        # Convert image to HSV
        # 将图像转换为HSV
        # 将图像转换为HSV
        HSV_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
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
            area = cv.contourArea(cnt)
            if area > 1000:
                point_x = float(x + w / 2)
                point_y = float(y + h / 2)
                cv.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.circle(self.image, (int(point_x), int(point_y)), 5, (0, 0, 255), -1)
                cv.putText(self.image, self.color_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                # Calculate the position of the block in the image
                # 计算方块在图像中的位置
                a = round(((point_x - 320) / 4000)-0.0025, 5)
                b = round(0.175-((point_y-240) / 3000 * 0.8), 5)
                angle = -math.atan2(a, b)*57.3*0.5
                return (a, b, angle)

    def target_run(self, msg, xy=None):
        '''
        grab function 抓取函数
        :param msg: (颜色,位置)  (color, position)
        '''
        if xy != None: self.xy = xy
        move_status=0
        for i in msg.values():
            if i !=None: move_status=1
        if move_status==1:
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)
        for name, pos in msg.items():
            # Here, ROS inversely solves the communication to obtain the rotation angle of each joint
            # 此处ROS反解通讯,获取各关节旋转角度
            joints = self.server_joint(pos)
            self.grap.identify_move(str(name), joints)
        if move_status==1:
            # set up
            # 架起
            joints_uu = [90, 80, 50, 50, 90, 30]
            # Move over the object's position
            # 移动至物体位置上方
            self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
            sleep(1)
            # initial position
            # 初始位置
            joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]
            # move to initial position
            # 移动至初始位置
            self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
            sleep(1.5)

    def server_joint(self, posxy):
        '''
        Post position request, get joint rotation angle
        发布位置请求,获取关节旋转角度
        :param posxy: 位置点x,y坐标 Location point x,y coordinates
        :return: 每个关节旋转角度    Rotation angle of each joint
        '''
        # Create a message pack
        # 创建消息包
        request = Kinemarics.Request()
        request.tar_x = posxy[0]
        request.tar_y = posxy[1]
        request.tar_z = 0.02
        request.roll = -1.57
        request.pitch = 0.0
        request.yaw = 0.0
        request.kin_name = "ik"
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            # Get the inverse solution response result
            # 获得反解响应结果
            joints = [0.0, 0.0, 0.0, 0.0, 0.0]
            joints[0] = response.joint1
            joints[1] = response.joint2
            joints[2] = response.joint3
            joints[3] = response.joint4
            joints[4] = posxy[2] + 90
            return joints
        else:
            self.get_logger().info('arg error')
