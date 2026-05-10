#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep


class stacking_grap:
    def __init__(self):
        # set move status
        # 设置移动状态
        self.move_status = True
        self.arm = Arm_Lib.Arm_Device()
        # Clamping jaw tightening angle
        # 夹爪加紧角度
        self.grap_joint = 140

    def move(self, joints, joints_down):
        '''
        Moving process
        移动过程
        :param joints: 移动到物体位置的各关节角度   The angle of each joint moved to the position of the object
        :param joints_down: 机械臂堆叠各关节角度   Manipulator stacking joint angle
        '''
        joints_0 = [90, 90, 15, 20, 90, 30]
        #joints = [90, 40, 30, 67, 265, 30]
        joints_uu = [90, 75, 50, 50, 265, self.grap_joint]
        # put up 抬起
        joints_up = [joints_down[0], 75, 50, 50, 265, 30]
        # Move over the object's position 移动至物体位置上方
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # Release the jaws 松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # move to object position 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 500)
        sleep(0.5)
        # gripping, clamping jaws 进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # set up 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # Lift to the top of the corresponding position 抬起至对应位置上方
        self.arm.Arm_serial_servo_write(1, joints_down[0], 500)
        sleep(0.5)
        # Lift to the corresponding position 抬起至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1)
        # Release the object, release the gripper释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # put up 抬起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # move to initial position 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)

    def arm_run(self, move_num, joints):
        '''
        Manipulator movement function 机械臂移动函数
        :param move_num: 抓取次数   Grab times
        :param joints: 反解求得的各关节角度 Angle of each joint obtained by inverse solution
        '''
        # a=[132, 50, 20, 60, 265, 100]
        # b=[132, 55, 38, 38, 265, 100]
        # c=[132, 60, 45, 30, 265, 100]
        # d=[132, 65, 55, 20, 265, 100]
        if move_num == '1' and self.move_status == True:
            # It is set here. You can only run down after this operation
            # 此处设置,需执行完本次操作,才能向下运行
            self.move_status = False
            # print(joints[0], joints[1], joints[2], joints[3], joints[4])
            # Obtain the target joint angle 获得目标关节角
            joints = [joints[0], joints[1], joints[2], joints[3]+4, 265, 30]
            # Attitude before moving to the target 移动到目标前的姿态
            joints_down = [137, 50, 20, 60, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        if move_num == '2' and self.move_status == True:
            self.move_status = False
            # print joints[0], joints[1], joints[2], joints[3], joints[4]
            joints = [joints[0], joints[1], joints[2], joints[3]+4, 265, 30]
            joints_down = [137, 55, 38, 38, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        if move_num == '3' and self.move_status == True:
            self.move_status = False
            # print joints[0], joints[1], joints[2], joints[3], joints[4]
            joints = [joints[0], joints[1], joints[2], joints[3]+4, 265, 30]
            joints_down = [137, 60, 45, 30, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        if move_num == '4' and self.move_status == True:
            self.move_status = False
            # print joints[0], joints[1], joints[2], joints[3], joints[4]
            joints = [joints[0], joints[1], joints[2], joints[3]+4, 265, 30]
            joints_down = [137, 65, 55, 20, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
