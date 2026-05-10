#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep
from dofbot_utils.robot_controller import Robot_Controller

class stacking_grap:
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
        :param joints_down: 机械臂堆叠各关节角度   Manipulator stacking joint angle
        '''
        joints_00 = [90, 80, 50, 50, self._joint_5, self.grap_joint]
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_00, 1500)
        sleep(2)
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
        self.arm.Arm_serial_servo_write6_array(joints_00, 1000)
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


    def arm_run(self, move_num, joints):
        '''
        Manipulator movement function 机械臂移动函数
        :param move_num: 抓取次数   Grab times
        :param joints: 反解求得的各关节角度 Angle of each joint obtained by inverse solution
        '''
        # Obtain the target joint angle 获得目标关节角
        joints = [joints[0], joints[1], joints[2], joints[3], joints[4], 30]
        if move_num == '1' and self.move_status == True:
            # It is set here. You can only run down after this operation
            # 此处设置,需执行完本次操作,才能向下运行
            self.move_status = False
            # Attitude before moving to the target 移动到目标前的姿态
            joints_down = self.robot.P_HEAP_1
            self.move(joints, joints_down)
            self.move_status = True
        if move_num == '2' and self.move_status == True:
            self.move_status = False
            joints_down = self.robot.P_HEAP_2
            self.move(joints, joints_down)
            self.move_status = True
        if move_num == '3' and self.move_status == True:
            self.move_status = False
            joints_down = self.robot.P_HEAP_3
            self.move(joints, joints_down)
            self.move_status = True
        if move_num == '4' and self.move_status == True:
            self.move_status = False
            joints_down = self.robot.P_HEAP_4
            self.move(joints, joints_down)
            self.move_status = True
