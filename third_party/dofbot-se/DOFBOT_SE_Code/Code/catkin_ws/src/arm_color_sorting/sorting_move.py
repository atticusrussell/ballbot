#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep


class sorting_move:
    def __init__(self):
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        # 夹爪加紧角度
        self.grap_joint = 135
        # 夹取位置
#         self.joints = [90, 55, 30, 35, 90, 0]
        self.joints = [90, 53, 33, 36, 90, 0]
        
    def sorting_move(self, joints_target):
        '''
        移动过程
        '''
        joints_up = [90, 80, 35, 40, 90, self.grap_joint]
        # 架起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # 松开夹爪
        self.arm.Arm_serial_servo_write(6, 0, 500)
        sleep(0.5)
        # 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(self.joints, 1000)
        sleep(1)
        # 夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # 架起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # 旋转
        self.arm.Arm_serial_servo_write(1, joints_target[0], 500)
        sleep(0.5)
        # 移动至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_target, 1000)
        sleep(1.5)
        # 松开夹爪
        self.arm.Arm_serial_servo_write(6, 0, 500)
        sleep(0.5)
        # 抬起
        joints_up[0] = joints_target[0]
        self.arm.Arm_serial_servo_write6_array(joints_up, 500)
        sleep(0.5)
        # 返回至中心位置
        self.arm.Arm_serial_servo_write(1, 90, 500)
        sleep(0.5)
        # 初始位置
        joints_0 = [90, 135, 0, 0, 90, 0]
        # 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
        sleep(1)

    def sorting_run(self, name):
        '''
        机械臂移动函数
        '''
        if name == "red" :
            # print("red")
            # 物体放置位姿
#             joints_target = [115, 20, 80, 40, 90, self.grap_joint]
            joints_target = [117, 19, 66, 56, 90, self.grap_joint]
            # 移动
            self.sorting_move(joints_target)
        if name == "blue":
            # print("blue")
#             joints_target = [45, 80, 0, 40, 90, self.grap_joint]
            joints_target = [44, 66, 20, 28, 90, self.grap_joint]
            self.sorting_move(joints_target)
        if name == "green" :
            # print("green")
#             joints_target = [137, 80, 0, 40, 90, self.grap_joint]
            joints_target = [136, 66, 20, 29, 90, self.grap_joint]
            self.sorting_move(joints_target)
        if name == "yellow" :
            # print("yellow")
#             joints_target = [65, 20, 80, 40, 90, self.grap_joint]
            joints_target = [65, 22, 64, 56, 90, self.grap_joint]
            self.sorting_move(joints_target)
