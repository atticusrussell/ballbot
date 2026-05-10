#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep


class snake_move:
    def __init__(self):
        self.arm = Arm_Lib.Arm_Device()
        # 夹爪加紧角度 Gripper tightening angle
        self.grap_joint = 140

    def arm_move(self, joints_target):

        self.arm.Arm_Buzzer_On(1)
        # 转平 turn flat
        sleep(1)
        self.arm.Arm_serial_servo_write(5, 90, 500)
        sleep(1)
        # 松开夹爪 Release the jaws
        self.arm.Arm_serial_servo_write(6, 20, 500)
        sleep(1.5)
        # 夹紧夹爪 Clamping jaws
        self.arm.Arm_serial_servo_write(6, 140, 500)
        sleep(0.8)
        # 移动至对应位置 move to the corresponding location
        self.arm.Arm_serial_servo_write6_array(joints_target, 1000)
        sleep(1.5)
        # 松开夹爪 Release the jaws
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # 回到初始位置 return to original position
        joint_00 = [90, 135, 0, 45, 0, 180]
        self.arm.Arm_serial_servo_write6_array(joint_00, 500)
        sleep(1)

    def snake_run(self, name):
        if name == "red":
            #print("red")
            # 物体放置位姿 object placement pose
            joints_target = [117, 19, 66, 56, 90, self.grap_joint]
            self.arm_move(joints_target)
        if name == "blue":
            # print("blue")
            joints_target = [44, 66, 20, 28, 90, self.grap_joint]
            self.arm_move(joints_target)
        if name == "green":
            # print("green")
            joints_target = [136, 66, 20, 29, 90, self.grap_joint]
            self.arm_move(joints_target)
        if name == "yellow":
            print("yellow")
            joints_target = [65, 22, 64, 56, 90, self.grap_joint]
            self.arm_move(joints_target)
