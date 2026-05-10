# !/usr/bin/env python3
# coding: utf-8
import math
import rclpy
from rclpy.node import Node
import threading
import Arm_Lib
from time import sleep
from snake_move import snake_move
from dofbot_pro_interface.srv import *
#from dofbot_pro_interface.srv import *
rclpy.init()
import time

class snake_ctrl(Node):
    def __init__(self):
        super().__init__('dofbot_snake')
        self.sbus = Arm_Lib.Arm_Device()
        self.arm_move = snake_move()
        self.color_name = None
        self.image = None
        self.text_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_time = 1000
        # 设置移动状态 set mobile state
        self.grap_status = 'Waiting'
        self.num = 1
        self.move_num = 1
        # Creates a handle to the ROS service to invoke.
        # 创建用于调用的ROS服务的句柄
        self.client = self.create_client(Kinemarics, "dofbot_kinemarics")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def read_joint(self):
        '''
        Loop to read the current angle of the servo
        循环读取舵机的当前角度
        '''
        num = 0
        for i in range(1, 6):
            while 1:
                # 读取舵机角度 Read the servo angle
                joint = self.sbus.Arm_serial_servo_read(i)
                # Whenever data is read, jump out of the loop and return the result
                # 每当读取到数据时,跳出循环,返回结果
                if joint != None:
                    self.cur_joint[i - 1] = joint
                    break
                num += 1
        # print("Current joint angle : {}".format(self.cur_joint))

    def get_Posture(self):
        '''
        Publish joint angle, get position
        发布关节角度,获取位置
        '''
        self.read_joint()
        # 创建消息包 Create a message pack
        request = Kinemarics.Request()
        request.cur_joint1 = float(self.cur_joint[0])
        request.cur_joint2 = float(self.cur_joint[1])
        request.cur_joint3 = float(self.cur_joint[2])
        request.cur_joint4 = float(self.cur_joint[3])
        request.cur_joint5 = float(self.cur_joint[4])
        request.kin_name = "fk"
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            # 获得反解响应结果 Get the inverse solution response result
            self.Posture[0] = response.x
            self.Posture[1] = response.y
            self.Posture[2] = response.z
            self.Posture[3] = response.roll
            self.Posture[4] = response.pitch
            self.Posture[5] = response.yaw
            # print(f"正解服务结果: x={self.Posture[0]}, y={self.Posture[1]}, z={self.Posture[2]}, roll={self.Posture[3]}, pitch={self.Posture[4]}, yaw={self.Posture[5]}")
        else:
            self.get_logger().info('get_Posture error')

    def joints_limit(self, joints):
        # 创建消息包 Create a message pack
        request = Kinemarics.Request()
        request.cur_joint1 = joints[0]
        request.cur_joint2 = joints[1]
        request.cur_joint3 = joints[2]
        request.cur_joint4 = joints[3]
        request.cur_joint5 = joints[4]
        request.kin_name = "fk"
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        # print(joints[3])
        if joints[2] < 0:
            joints[2]  = 0
        if joints[3] < 20:
            joints[3]  = 18
        if joints[1] > 145:
            joints[1]  = 145

        if joints[1] < 30:
            joints[1]  = 30
        if joints[2] > 90:
            joints[2]  = 90
        if joints[3] > 60:
            joints[3]  = 60
        
        # 
        # print(response.y)
        if response is not None:
            if 0.22 < response.z < 0.23 and response.y > 0.145 :
                move_joint1 = [90, joints[1], joints[2], joints[3], 0, 180]
                move_joint2 = [90, joints[1], joints[2], joints[3], 0, 30]
                if self.move_num == 1:
                    self.sbus.Arm_serial_servo_write6_array(move_joint1, self.move_time)
                    self.move_num = 2
                if self.move_num == 2:
                    self.sbus.Arm_serial_servo_write6_array(move_joint2, self.move_time)
                    self.move_num = 1
        else:
            self.get_logger().info("joints_limit error")

    def snake_run(self, point_y):
        '''
        Post position request, get joint rotation angle
        发布位置请求,获取关节旋转角度
        '''
       
        request = Kinemarics.Request()
        request.tar_x = self.Posture[0]
        request.tar_y = point_y
        request.tar_z = 0.225476
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
            joints[4] = response.joint5
            
            self.text_joint[0] = joints[0]
            self.text_joint[1] = joints[1]
            self.text_joint[2] = joints[2]
            self.text_joint[3] = joints[3]
            self.text_joint[4] = joints[4]

            self.joints_limit(joints)
            # print(self.text_joint)
        else:
            self.get_logger().info("snake_run error")

    def snake_main(self, name, msg):
  
        for key, area in msg.items():
            if key == name:
                # Estimate the position of the block according to the camera
                # 估计方块据摄像头的位置
                distance = 27.05 * math.pow(area, -0.51) - 0.2
                # Estimate the position of the block in the world coordinate system
                # 估计方块在世界坐标系下的位置
                target_dist = distance + self.Posture[1]

                if self.grap_status == 'Waiting':
                    threading.Thread(target=self.snake_run, args=(target_dist,)).start()

                    if 145 < self.text_joint[1] <180 :
                        self.sbus.Arm_serial_servo_write(5,180,300)
                        sleep(0.2)
                        self.sbus.Arm_serial_servo_write(5,0, 300)
                        sleep(0.2)
                        self.num = 1
                    elif  10 < self.text_joint[1] <14 and 89.5< self.text_joint[2] <91:
                        # Gripper opening and closing
                        # print("夹爪张合")
                        self.num += 1
                    else:
                        self.num = 1
                    if self.num % 30 == 0: self.grap_status = 'Graping'
                elif self.grap_status == 'Graping':
                    self.sbus.Arm_Buzzer_On(1) 
                    self.grap_status = 'Runing'
                    # 执行放下 put down
                    self.arm_move.snake_run(name)
                    # 动作完毕 action completed
                    self.num = 1
                    # 设置移动状态 set mobile state
                    self.grap_status = 'Waiting'

    

