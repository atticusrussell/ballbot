#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep


class single_garbage_move:
    def __init__(self):
        # 设置移动状态
        self.move_status = True
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        # 夹爪加紧角度
        self.grap_joint = 135

    def move(self, joints_down):
        '''
        移动过程
        :param joints_down: 机械臂抬起各关节角度
        :param color_angle: 移动到对应垃圾桶的角度
        '''
        # 初始位置
        joints_0 = [90, 135, 0, 0, 90, 0]
        # 抓取角度
        joints=[90, 45, 30, 65, 265 , 0]
        joints_uu = [90, 80, 50, 50, 265, self.grap_joint]
        # 抬起
        joints_up = [joints_down[0], 80, 50, 50, 265, 0]
        # 移动至物体位置上方
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # 松开夹爪
        self.arm.Arm_serial_servo_write(6, 0, 500)
        sleep(0.5)
        # 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 500)
        sleep(0.5)
        # 进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # 抬起至对应位置上方
        self.arm.Arm_serial_servo_write(1, joints_down[0], 500)
        sleep(0.5)
        # 抬起至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1)
        # 释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 0, 500)
        sleep(0.5)
        # 抬起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)

    def single_garbage_grap(self, name):
        '''
        机械臂移动函数
        :param name:识别的垃圾类别
        '''
        self.arm.Arm_Buzzer_On(1)
        sleep(0.5)
        # 有害垃圾--红色 04
        if name == "04" and self.move_status == True:
            # 此处设置,需执行完本次操作,才能向下运行
            self.move_status = False
            # print("有害垃圾")
            # 移动到垃圾桶前对应姿态
            # joints_down = [45, 80, 35, 40, 265, self.grap_joint]
            # 移动到垃圾桶位置放下对应姿态
            joints_down = [45, 50, 20, 60, 265, self.grap_joint]
            # 移动
            self.move(joints_down)
            # 移动完毕
            self.move_status = True
        # 可回收垃圾--蓝色 01
        if name == "01" and self.move_status == True:
            self.move_status = False
            # print("可回收垃圾")
            # joints_down = [27, 110, 0, 40, 265, self.grap_joint]
            joints_down = [27, 75, 0, 50, 265, self.grap_joint]
            self.move(joints_down)
            self.move_status = True
        # 厨余垃圾--绿色 03
        if name == "03" and self.move_status == True:
            self.move_status = False
            # print("厨余垃圾")
            # joints_down = [152, 110, 0, 40, 265, self.grap_joint]
            joints_down = [147, 75, 0, 50, 265, self.grap_joint]
            self.move(joints_down)
            self.move_status = True
        # 其他垃圾--灰色 02
        if name == "02" and self.move_status == True:
            self.move_status = False
            # print("其他垃圾")
            # joints_down = [137, 80, 35, 40, 265, self.grap_joint]
            joints_down = [133, 50, 20, 60, 265, self.grap_joint]
            self.move(joints_down)
            self.move_status = True
