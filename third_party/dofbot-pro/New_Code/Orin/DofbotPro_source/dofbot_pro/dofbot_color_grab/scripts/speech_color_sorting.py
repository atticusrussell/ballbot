#!/usr/bin/env python
# coding: utf-8
import random
import Arm_Lib
import threading
import cv2 as cv
import time
from time import sleep
import smbus
from dofbot_utils.robot_controller import Robot_Controller

from Speech_Lib import Speech
mySpeech = Speech()

  


class speech_color_sorting:
    def __init__(self):
        '''
        设置初始化参数
        '''
        self.image = None
        # 初始化计数器
        self.num = 0
        # 初始化运动状态
        self.status = 'waiting'
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        self.robot = Robot_Controller()
        # 夹爪加紧角度
        self.grap_joint = self.robot.get_gripper_value(1)
        # 夹取位置
        self.joints = self.robot.P_CENTER
        self.joints[5] = self.robot.get_gripper_value(0)


    def get_Sqaure(self, color_name, hsv_lu):
        '''
        颜色识别
        '''
        (lowerb, upperb) = hsv_lu
        # 设置识别区域
        point_Xmin=50   #200
        point_Xmax=600  #440
        point_Ymin=80   #200
        point_Ymax=480  #480
        # 画矩形框
        # cv.rectangle(self.image, (point_Xmin, point_Ymin), (point_Xmax, point_Ymax),(105,105,105), 2)
        # 复制原始图像,避免处理过程中干扰
        img = self.image.copy()
        mask = img[point_Ymin:point_Ymax, point_Xmin:point_Xmax]

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
                cv.rectangle(self.image, (x + point_Xmin, y + point_Ymin), (x + w + point_Xmin, y + h + point_Ymin), (0, 255, 0), 2)
                # 绘制矩形中心
                cv.circle(self.image, (int(x_w_ + point_Xmin), int(y_h_ + point_Ymin)), 5, (0, 0, 255), -1)
                # # 在图片中绘制结果
                cv.putText(self.image, color_name, (int(x + point_Xmin-15), int(y + point_Ymin-15)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                
                # cv.rectangle(self.image, (x , y ), (x + w , y + h ), (0, 255, 0), 2)
                # cv.circle(self.image, (int(x_w_ ), int(y_h_ )), 5, (0, 0, 255), -1)
                # cv.putText(self.image, color_name, (int(x -15), int(y -15)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                return (x_w_, y_h_)

    def Sorting_grap(self, img, color_hsv):
        # 规范输入图像大小
        self.image = cv.resize(img, (640, 480))
        # 获取识别的结果
        msg = {}
        if self.status == 'waiting':
            # 遍历颜色通道,获取能够识别的结果
            for key, value in color_hsv.items():
                point = self.get_Sqaure(key, value)
                if point != None: msg["name"] = key
            if len(msg) == 1:
                self.num += 1
                # 每当连续识别10次并且运动状态为waiting的情况下,执行抓取任务
                if self.num % 10 == 0 and self.status == 'waiting':
                    self.status = "Runing"
                    self.arm.Arm_Buzzer_On(1)
                    sleep(0.5)
                    # 开启抓取线程
                    threading.Thread(target=self.sorting_run, args=(msg['name'],)).start()
                    self.num = 0
        return self.image

    def sorting_move(self, joints_target):
        '''
        移动过程
        '''
        joints_up = self.robot.P_TOP
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
        self.arm.Arm_serial_servo_write(1, joints_target[0], 1000)
        sleep(1)
        # Move to corresponding position 移动至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_target, 1000)
        sleep(1.5)
        # Release the object and release the clamping jaws释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.8)
        # raise  抬起
        self.arm.Arm_serial_servo_write(2, 90, 1000)
        sleep(1)
        # return to center 返回至中心位置
        self.arm.Arm_serial_servo_write(1, 90, 1000)
        sleep(1)
        # move to initial position 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_MAP, 1000)
        sleep(1.5)

    def sorting_run(self, name):
        '''
        机械臂移动函数
        '''
        if name == "red" :            
            mySpeech.void_write(88)
            time.sleep(1.0)
            mySpeech.void_write(46)            
            # print("red")
            self.sorting_move(self.robot.P_RED)
            mySpeech.void_write(81)  
            # 抓取完毕
            self.status = 'waiting'
        if name == "blue":
            mySpeech.void_write(88)
            time.sleep(1.0)
            mySpeech.void_write(48) 
            # print("blue")
            self.sorting_move(self.robot.P_BLUE)
            mySpeech.void_write(81) 
            # 抓取完毕
            self.status = 'waiting'
        if name == "green" :
            mySpeech.void_write(88) 
            time.sleep(1.0)
            mySpeech.void_write(49) 
            # print("green")
            self.sorting_move(self.robot.P_GREEN)
            mySpeech.void_write(81)
            # 抓取完毕
            self.status = 'waiting'
        if name == "yellow" :
            mySpeech.void_write(88)
            time.sleep(1.0)
            mySpeech.void_write(47)  
            # print("yellow")
            self.sorting_move(self.robot.P_YELLOW)
            mySpeech.void_write(81)  
            # 抓取完毕
            self.status = 'waiting'


