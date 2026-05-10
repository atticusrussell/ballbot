#!/usr/bin/env python3
# coding: utf-8
import time
import rclpy
from rclpy.node import Node
import Arm_Lib
import cv2 as cv
import numpy as np
from time import sleep
from numpy import random
from ultralytics import YOLO
import torch
from garbage_grap import garbage_grap_move
from dofbot_pro_interface.srv import *
import math
rclpy.init()

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Load the YOLOv11 model
model_path = '/home/jetson/ultralytics/ultralytics/data/yahboom_data/best.engine'
model = YOLO(model_path)

# Get names and colors
names = model.names
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]


class garbage_identify(Node):
    def __init__(self):
        super().__init__('dofbot_garbage')
        self.frame = None
        self.arm = Arm_Lib.Arm_Device()
        # Robotic arm recognition position adjustment
        # 机械臂识别位置调节
        self.xy = [90, 130]
        self.garbage_index = 0
        self.grap_move = garbage_grap_move()
        # Creates a handle to the ROS service to invoke.
        # 创建用于调用的ROS服务的句柄
        self.client = self.create_client(Kinemarics, "dofbot_kinemarics")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def plot_one_box(self, x, img, color=None, label=None, line_thickness=3):
        # Plots one bounding box on image img
        tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv.rectangle(img, c1, c2, color, thickness=tl, lineType=cv.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv.rectangle(img, c1, c2, color, -1, cv.LINE_AA)  # filled
            cv.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv.LINE_AA)

    def garbage_grap(self, msg, xy=None):
        '''
        Execute grab function
        执行抓取函数
        :param msg: {name:pos,...}
        '''
        if xy is not None:
            self.xy = xy
        if len(msg) != 0:
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)
        for index, name in enumerate(msg):
            joints = self.server_joint(msg[name])
            self.grap_move.arm_run(str(name), joints)
            # try:
            #     # Here, ROS inversely solves the communication to obtain the rotation angle of each joint
            #     # 此处ROS反解通讯,获取各关节旋转角度
            #     joints = self.server_joint(msg[name])
            #     # print(joints)
            #     # call the move function
            #     # 调取移动函数
            #     self.grap_move.arm_run(str(name), joints)
            # except Exception:
            #     print("sqaure_pos empty")
        joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]
        # move to initial position
        # 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
        sleep(1)

    def garbage_run(self, image):
        '''
        执行垃圾识别函数  Execute the garbage identification function
        :param image: 原始图像     The original image
        :return: 识别后的图像,识别信息(name, pos) Recognized image, identification information (name, pos)
        '''
        self.frame = cv.resize(image, (640, 480))
        msg={}
        # get identifying message
        # 获取识别消息
        # try: msg = self.get_pos()
        # except Exception: print("get_pos NoneType")
        msg = self.get_pos(self.frame)
        return self.frame, msg

    def get_pos(self, image):
        '''
        获取识别信息 Obtain identifying information
        :return: 名称,位置 name, location
        '''
        # Copy the original image to avoid interference during processing
        # 复制原始图像,避免处理过程中干扰
        img = image.copy()
        # Inference
        results = model(img, verbose=False)
        msg = {}
        if results:
            for box in results[0].boxes:
                # 从 box 中获取归一化的 xywh 信息
                xywh = box.xywhn.view(-1).tolist()
                conf = box.conf.item()
                cls = int(box.cls.item())
                prediction_status = True
                label = '%s %.2f' % (model.names[cls], conf)
                #print(label)
                # get name
                name = names[int(cls)]
                name_list = ["Vegetable_leaf", "Banana_peel", "Shell", "Plastic_bottle", "Basketball", "Carton", "Bandage", "Expired_capsule_drugs"]
                for i in name_list:
                    if name == i:
                        prediction_status = False
                if prediction_status and conf > 0.65:
                    point_x = np.int(xywh[0] * 640)
                    point_y = np.int(xywh[1] * 480)
                    cv.circle(self.frame, (point_x, point_y), 5, (0, 0, 255), -1)
                    self.plot_one_box(box.xyxy.view(-1).tolist(), self.frame, label=label, color=colors[int(cls)], line_thickness=2)
                    # 计算方块在图像中的位置
                    # (a, b) = (round(((point_x - 320) / 4000), 5), round(((480 - point_y) / 3000) * 0.8+0.18, 5))
                    # msg[name] = (a, b)
                    a = round(((point_x - 320) / 4000) - 0.0025, 5)
                    b = round(0.175 - ((point_y - 240) / 3000 * 0.8), 5)
                    angle = -math.atan2(a, b) * 57.3 * 0.5
                    msg[name] = (a, b, angle)
        return msg

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
        print("request:", request.tar_x, request.tar_y)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            # Get the response result of the inverse solution
            # 获取反解的响应结果
            joints = [0, 0, 0, 0, 0]
            joints[0] = response.joint1
            joints[1] = response.joint2
            joints[2] = response.joint3
            joints[3] = response.joint4
            # joints[4] = response.joint5
            joints[4] = posxy[2] + 90
            print("joints:", joints)
            # Angle adjustment
            # 角度调整
            if joints[2] < 0:
                joints[1] += joints[2] / 2
                joints[3] += joints[2] * 3 / 4
                joints[2] = 0
            # print joints
            return joints
        else:
            self.get_logger().info('Service call failed')


