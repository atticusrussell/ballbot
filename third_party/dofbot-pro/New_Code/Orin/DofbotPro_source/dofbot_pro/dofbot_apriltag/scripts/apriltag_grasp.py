#!/usr/bin/env python3
# coding: utf-8
import rclpy
from rclpy.node import Node
import Arm_Lib
import cv2 as cv
import numpy as np
from time import sleep
from dofbot_pro_interface.srv import *
from dofbot_utils.robot_controller import Robot_Controller
import math

rclpy.init()

class Apriltag_Grasp(Node):
    def __init__(self):
        super().__init__('dofbot_apritag')
        # Robotic arm recognition position adjustment
        # 机械臂识别位置调节
        self.xy = [90, 106]
       # set move status
        # 设置移动状态
        self.move_status = True
        self.arm = Arm_Lib.Arm_Device()
        self.robot = Robot_Controller()
        # Clamping jaw tightening angle
        # 夹爪加紧角度
        self.grap_joint = self.robot.get_gripper_value(1)
        self._joint_5 = self.robot.joint5
        # Creates a handle to the ROS service to invoke.
        # 创建用于调用的ROS服务的句柄
        self.client = self.create_client(Kinemarics, "dofbot_kinemarics")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')


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
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            # Get the inverse solution response result
            # 获得反解响应结果
            joints = [0.0, 0.0, 0.0, 0.0, 0.0]
            joints[0] = response.joint1
            joints[1] = response.joint2
            joints[2] = response.joint3
            joints[3] = response.joint4
            # joints[4] = response.joint5
            joints[4] = posxy[2] + 90
            return joints
        else:
            self.get_logger().info('Service call failed')

    def target_sorting(self, msg, xy=None):
        '''
        grab function 抓取函数
        :param msg: (ID,位置)  (ID, position)
        '''
        if xy != None: self.xy = xy
        move_status=0
        for i in msg.values():
            if i !=None: move_status=1
        if move_status==1:
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)
        for name, pos in msg.items():
            # Here, ROS inversely solves the communication to obtain the rotation angle of each joint
            # 此处ROS反解通讯,获取各关节旋转角度
            joints = self.server_joint(pos)
            self.sorting_move(str(name), joints)

        if move_status==1:
            # set up
            # 架起
            joints_uu = [90, 80, 50, 50, 90, 30]
            # Move over the object's position
            # 移动至物体位置上方
            self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
            sleep(1.5)
            # initial position
            # 初始位置
            joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]
            # move to initial position
            # 移动至初始位置
            self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
            sleep(1.5)



    def sorting_block(self, joints, joints_down):
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

    def sorting_move(self, name, joints):
        '''
        Manipulator movement function 机械臂移动函数
        :param name:识别的颜色  Recognized color
        :param joints: 反解求得的各关节角度 Angle of each joint obtained by inverse solution
        '''
        joints = [joints[0], joints[1], joints[2], joints[3], joints[4], 30]
        if name == "1" and self.move_status == True:
            # It is set here. You can only run down after this operation
            # 此处设置,需执行完本次操作,才能向下运行
            self.move_status = False
            self.sorting_block(joints, self.robot.P_NUM_1)
            self.move_status = True
        if name == "2" and self.move_status == True:
            self.move_status = False
            self.sorting_block(joints, self.robot.P_NUM_2)
            self.move_status = True
        if name == "3" and self.move_status == True:
            self.move_status = False
            self.sorting_block(joints, self.robot.P_NUM_3)
            self.move_status = True
        if name == "4" and self.move_status == True:
            self.move_status = False
            self.sorting_block(joints, self.robot.P_NUM_4)
            self.move_status = True


    
    def target_stacking(self, msg, xy=None):
        '''
        grab function 抓取函数
        :param msg: (ID,位置)  (ID, position)
        '''
        if xy != None: self.xy = xy
        move_status=0
        for i in msg.values():
            if i !=None: move_status=1
        if move_status==1:
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)
        stacking_num = 1
        for name, pos in msg.items():
            # Here, ROS inversely solves the communication to obtain the rotation angle of each joint
            # 此处ROS反解通讯,获取各关节旋转角度
            joints = self.server_joint(pos)
            self.stacking_move(stacking_num, str(name), joints)
            stacking_num = stacking_num + 1

        if move_status==1:
            # set up
            # 架起
            joints_uu = [90, 80, 50, 50, 90, 30]
            # Move over the object's position
            # 移动至物体位置上方
            self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
            sleep(1.5)
            # initial position
            # 初始位置
            joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]
            # move to initial position
            # 移动至初始位置
            self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
            sleep(1.5)
            
    def stacking_move(self, level, name, joints):
        '''
        Manipulator movement function 机械臂移动函数
        :param name:识别的颜色  Recognized color
        :param joints: 反解求得的各关节角度 Angle of each joint obtained by inverse solution
        '''
        joints = [joints[0], joints[1], joints[2], joints[3], joints[4], 30]
        
        if level == 1 and self.move_status == True:
            # It is set here. You can only run down after this operation
            # 此处设置,需执行完本次操作,才能向下运行
            self.move_status = False
            self.stacking_block(joints, self.robot.P_HEAP_1)
            self.move_status = True
        if level == 2 and self.move_status == True:
            self.move_status = False
            self.stacking_block(joints, self.robot.P_HEAP_2)
            self.move_status = True
        if level == 3 and self.move_status == True:
            self.move_status = False
            self.stacking_block(joints, self.robot.P_HEAP_3)
            self.move_status = True
        if level == 4 and self.move_status == True:
            self.move_status = False
            self.stacking_block(joints, self.robot.P_HEAP_4)
            self.move_status = True

    def stacking_block(self, joints, joints_down):
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
