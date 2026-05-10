# !/usr/bin/env python
# coding: utf-8
import cv2
import numpy as np
import random
import time
from Arm_Lib import Arm_Device
import threading

class color_grab:
    def __init__(self):
        '''
        初始化一些参数
        '''
        self.Arm = Arm_Device()
        self.color_name = None
        self.image = None

        self.look_at = [90, 164, 18, 0, 90, 90]
        self.p_top = [90, 80, 50, 50, 270]

        self.p_Yellow = [65, 22, 64, 56, 270]
        self.p_Red = [118, 19, 66, 56, 270]

        self.p_Green = [136, 66, 20, 29, 270]
        self.p_Blue = [44, 66, 20, 28, 270]

        self.p_gray = [90, 48, 35, 30, 270]

        self.g_state_arm = 0
        self.started = 0

        

    # 定义移动机械臂函数,同时控制1-5号舵机运动，p=[S1,S2,S3,S4,S5]
    def arm_move(self, p, s_time = 500):
        for i in range(5):
            id = i + 1
            if id == 5:
                time.sleep(.1)
                self.Arm.Arm_serial_servo_write(id, p[i], int(s_time*1.2))
            elif id == 1 :
                self.Arm.Arm_serial_servo_write(id, p[i], int(3*s_time/4))
            else:
                self.Arm.Arm_serial_servo_write(id, p[i], int(s_time))
            time.sleep(.01)
        time.sleep(s_time/1000)
    
    # 定义夹积木块函数，enable=1：夹住，=0：松开
    def arm_clamp_block(self, enable):
        if enable == 0:
            self.Arm.Arm_serial_servo_write(6, 60, 400)
        else:
            self.Arm.Arm_serial_servo_write(6, 135, 400)
        time.sleep(.5)

    # 控制机械臂移动
    def ctrl_arm_move(self, index):
        self.arm_clamp_block(0)
        if index == 1:
            print("黄色")
            self.Arm.Arm_Buzzer_On(1)
            time.sleep(1)
            self.number_action(index)
            self.put_down_block()
        elif index == 2:
            print("红色")
            self.Arm.Arm_Buzzer_On(1)
            time.sleep(1)
            self.number_action(index)
            self.put_down_block()
        elif index == 3:
            print("绿色")
            self.Arm.Arm_Buzzer_On(1)
            time.sleep(1)
            self.number_action(index)
            self.put_down_block()
        elif index == 4:
            print("蓝色")
            self.Arm.Arm_Buzzer_On(1)
            time.sleep(1)
            self.number_action(index)
            self.put_down_block()
        self.g_state_arm = 0

    #数字功能定义
    def number_action(self, index):
        if index == 1:
            # 抓取黄色的积木块
            self.arm_move(self.p_top, 1000)
            self.arm_move(self.p_Yellow, 1000)
            self.arm_clamp_block(1)
    #         time.sleep(.5)
            self.arm_move(self.p_top, 1000)
        elif index == 2:
            # 抓取红色的积木块
            self.arm_move(self.p_top, 1000)
            self.arm_move(self.p_Red, 1000)
            self.arm_clamp_block(1)
            self.arm_move(self.p_top, 1000)
        elif index == 3:
            # 抓取绿色的积木块
            self.arm_move(self.p_top, 1000)
            self.arm_move(self.p_Green, 1000)
            self.arm_clamp_block(1)
            self.arm_move(self.p_top, 1000)
        elif index == 4:
            # 抓取蓝色的积木块
            self.arm_move(self.p_top, 1000)
            self.arm_move(self.p_Blue, 1000)
            self.arm_clamp_block(1)
            self.arm_move(self.p_top, 1000)

    # 放下积木    
    def put_down_block(self):
        self.arm_move(self.p_gray, 1000)
        self.arm_clamp_block(0) 
        self.Arm.Arm_serial_servo_write6_array(self.look_at, 1000)
        time.sleep(1)

    def start_move_arm(self, index):
        # 开启机械臂控制线程
        if self.g_state_arm == 0:
            closeTid = threading.Thread(target = self.ctrl_arm_move, args = [index])
            closeTid.setDaemon(True)
            closeTid.start()
            
            self.g_state_arm = 1
 

    def get_color(self, img):
        H = []
        color_name={}
        img = cv2.resize(img, (640, 480), )
        # 将彩色图转成HSV
        HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 画矩形框
        cv2.rectangle(img, (280, 180), (360, 260), (0, 255, 0), 2)
        # 依次取出每行每列的H,S,V值放入容器中
        for i in range(280, 360):
            for j in range(180, 260): H.append(HSV[j, i][0])
        # 分别计算出H,S,V的最大最小
        H_min = min(H);H_max = max(H)
    #     print(H_min,H_max)
        # 判断颜色
        if H_min >= 0 and H_max <= 10 or H_min >= 156 and H_max <= 180: color_name['name'] = 'red'
        elif H_min >= 26 and H_max <= 34: color_name['name'] = 'yellow'
        elif H_min >= 35 and H_max <= 78: color_name['name'] = 'green'
        elif H_min >= 100 and H_max <= 124: color_name['name'] = 'blue'
        return img, color_name

    def reset_state(self):
        self.started = 0


    def Color_Recongnize(self, frame):
        if self.started == 0:
            self.Arm.Arm_serial_servo_write6_array(self.look_at, 1000)
            time.sleep(1)

            self.Arm.Arm_Buzzer_On(1)
            s_time = 300
            self.Arm.Arm_serial_servo_write(4, 10, s_time)
            time.sleep(s_time/1000)
            self.Arm.Arm_serial_servo_write(4, 0, s_time)
            time.sleep(s_time/1000)
            self.Arm.Arm_serial_servo_write(4, 10, s_time)
            time.sleep(s_time/1000)
            self.Arm.Arm_serial_servo_write(4, 0, s_time)
            time.sleep(s_time/1000)
            self.started = 1
        
        
        frame, color_name = self.get_color(frame)
        if len(color_name)==1:
            # print ("name :", color_name['name'])
            if color_name['name'] == 'yellow':
#                 self.Arm.Arm_Buzzer_On(1)
                self.start_move_arm(1)
            elif color_name['name'] == 'red':
#                 self.Arm.Arm_Buzzer_On(1)
                self.start_move_arm(2)
            elif  color_name['name'] == 'green':
#                 self.Arm.Arm_Buzzer_On(1)
                self.start_move_arm(3)
            elif color_name['name'] == 'blue':
#                 self.Arm.Arm_Buzzer_On(1)
                self.start_move_arm(4)
                
        return frame
            

    def start_grab(self, img):
        '''
        颜色跟随控制函数
        :param img: 输入图像
        :return: 输出处理后的图像
        '''
        self.image = self.Color_Recongnize(img)

        return self.image

