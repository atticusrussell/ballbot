#!/usr/bin/env python
# coding: utf-8

import Arm_Lib
from time import sleep
from dofbot_utils.robot_controller import Robot_Controller

class identify_grap:
    def __init__(self):
        # set move status
        # 设置移动状态
        self.move_status = True
        self.arm = Arm_Lib.Arm_Device()
        self.robot = Robot_Controller()
        # Clamping jaw tightening angle
        # 夹爪加紧角度
        self.grap_joint = self.robot.get_gripper_value(1)
        self._joint_5 = self.robot.joint5

    def move(self, joints, joints_down):
        '''
        Moving process
        移动过程
        :param joints: 移动到物体位置的各关节角度   The angle of each joint moved to the position of the object
        :param joints_up: 机械臂抬起各关节角度     The manipulator raises the angle of each joint
        :param color_angle: 移动到对应垃圾桶的角度  Move to the angle of the corresponding trash can
        '''
        joints_uu = [90, 80, 50, 50, self._joint_5, self.grap_joint]
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1500)
        sleep(1)
        # Opening and closing jaws 开合夹爪
        for i in range(5):
            self.arm.Arm_serial_servo_write(6, 180, 100)
            sleep(0.1)
            self.arm.Arm_serial_servo_write(6, 30, 100)
            sleep(0.1)
        # Move to object position 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 1000)
        sleep(1.5)
        # Grasp and clamp the clamping claw进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(1)
        # 回退
        back_joints = [joints[0], joints[1]+30, joints[2]-30, joints[3]+10, joints[4], self.grap_joint]
        self.arm.Arm_serial_servo_write6_array(back_joints, 1000)
        sleep(1)
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1.5)
        # Lift to above the corresponding position 抬起至对应位置上方
        self.arm.Arm_serial_servo_write(1, joints_down[0], 1000)
        sleep(1.5)
        # Move to target location 移动至目标位置
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1.5)
        # Release the object and release the clamping jaws释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(1)
        # raise  抬起
        joints_up = [joints_down[0], 80, 50, 50, self._joint_5, 30]
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1.5)

    def identify_move(self, name, joints):
        '''
        Manipulator movement function 机械臂移动函数
        :param name:识别的颜色  Recognized color
        :param joints: 反解求得的各关节角度 Angle of each joint obtained by inverse solution
        '''
        joints = [joints[0], joints[1], joints[2], joints[3], joints[4], 30]
        if name == "red" and self.move_status == True:
            # It is set here. You can only run down after this operation
            # 此处设置,需执行完本次操作,才能向下运行
            self.move_status = False
            # print ("red")
            # joints_down = [45, 50, 20, 60, self._joint_5, self.grap_joint]
            # self.move(joints, joints_down)
            self.move(joints, self.robot.P_RED)
            self.move_status = True
        if name == "blue" and self.move_status == True:
            self.move_status = False
            # print ("blue")
            # joints_down = [27, 75, 0, 50, self._joint_5, self.grap_joint]
            # self.move(joints, joints_down)
            self.move(joints, self.robot.P_BLUE)
            self.move_status = True
        if name == "green" and self.move_status == True:
            self.move_status = False
            print ("green", self.robot.P_GREEN)
            # joints_down = [152, 75, 0, 50, self._joint_5, self.grap_joint]
            # self.move(joints, joints_down)
            self.move(joints, self.robot.P_GREEN)
            self.move_status = True
        if name == "yellow" and self.move_status == True:
            self.move_status = False
            # print ("yellow")
            # joints_down = [137, 50, 20, 60, self._joint_5, self.grap_joint]
            # self.move(joints, joints_down)
            self.move(joints, self.robot.P_YELLOW)
            self.move_status = True

