# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
from dofbot_utils import pid as PID
import Arm_Lib
import math


class Apriltag_Follow:
    def __init__(self):
        self.target_servox=90
        self.target_servoy=45
        self.Arm = Arm_Lib.Arm_Device()
        self.xservo_pid = PID.PositionalPID(1.3, 0.08, 0.15)
        self.yservo_pid = PID.PositionalPID(1.2, 0.05, 0.15)

    def follow_function(self, msg):
        if len(msg) == 0:
            return
        for name, pos in msg.items():
            point_x = pos[0]
            point_y = pos[1]
            if abs(point_x - 320) < 50 and abs(point_y - 240) < 40:
                return
            self.xservo_pid.SystemOutput = point_x
            self.xservo_pid.SetStepSignal(320)
            self.xservo_pid.SetInertiaTime(0.01, 0.1)
            if not (self.target_servox>=180 and point_x<=320 or self.target_servox<=0 and point_x>=320):
                self.xservo_pid.SystemOutput = point_x
                self.xservo_pid.SetStepSignal(320)
                self.xservo_pid.SetInertiaTime(0.01, 0.1)
                target_valuex = int(1500 + self.xservo_pid.SystemOutput)
                self.target_servox = int((target_valuex - 500) / 10)
                # Set movement restrictions
                # 设置移动限制
                if self.target_servox > 180:self.target_servox = 180
                if self.target_servox < 0: self.target_servox = 0
            if not (self.target_servoy>=180 and point_y<=240 or self.target_servoy<=0 and point_y>=240):
                # Input Y axis direction parameter PID control input
                # 输入Y轴方向参数PID控制输入
                self.yservo_pid.SystemOutput = point_y
                self.yservo_pid.SetStepSignal(240)
                self.yservo_pid.SetInertiaTime(0.01, 0.1)
                target_valuey = int(1500 + self.yservo_pid.SystemOutput)
                self.target_servoy = int((target_valuey - 500) / 10) - 45
                # Set movement restrictions
                # 设置移动限制
                if self.target_servoy > 360: self.target_servoy = 360
                if self.target_servoy < 0: self.target_servoy = 0
            joints_0 = [self.target_servox, 135, self.target_servoy / 2, self.target_servoy / 2, 90, 30]
            self.Arm.Arm_serial_servo_write6_array(joints_0, 1000)
            return


