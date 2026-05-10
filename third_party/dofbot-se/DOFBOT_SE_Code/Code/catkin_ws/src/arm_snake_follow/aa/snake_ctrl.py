# !/usr/bin/env python
# coding: utf-8
import math
import rospy
import threading
import Arm_Lib
from time import sleep
from snake_move import snake_move
from arm_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse


class snake_ctrl:
    def __init__(self):
        '''
        初始化一些参数
        '''
        self.sbus = Arm_Lib.Arm_Device()
        self.arm_move = snake_move()
        self.color_name = None
        self.image = None
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_time = 500
        # 设置移动状态
        self.grap_status = 'Waiting'
        # 夹爪加紧角度
        self.grap_joint = 145
        self.num = 1
        self.move_num = 1
        # 创建节点句柄
        self.n = rospy.init_node('ros_arm', anonymous=True)
        # 创建用于调用的ROS服务的句柄。
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)

    def read_joint(self):
        '''
        循环读取舵机的当前角度
        '''
        num = 0
        for i in range(1, 6):
            while 1:
                # 读取舵机角度
                joint = self.sbus.Arm_serial_servo_read(i)
                # 每十次读取不到数据,便打印警告
                # if num % 10 == 0: print("请检查!!!!!接触不良!!!!!")
                # 每当读取到数据时,跳出循环,返回结果
                if joint != None:
                    self.cur_joint[i - 1] = joint
                    break
                num += 1
        # 更新舵机为当前角度
        # print("当前关节角度 : {}".format(cur_joint))

    def get_Posture(self):
        '''
        发布关节角度,获取位置
        '''
        self.read_joint()
        # 等待server端启动
        self.client.wait_for_service()
        # 创建消息包
        request = kinemaricsRequest()
        request.cur_joint1 = self.cur_joint[0]
        request.cur_joint2 = self.cur_joint[1]
        request.cur_joint3 = self.cur_joint[2]
        request.cur_joint4 = self.cur_joint[3]
        request.cur_joint5 = self.cur_joint[4]
        request.kin_name = "fk"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):
                # 获得反解响应结果
                self.Posture[0] = response.x
                self.Posture[1] = response.y
                self.Posture[2] = response.z
                self.Posture[3] = response.Roll
                self.Posture[4] = response.Pitch
                self.Posture[5] = response.Yaw
        except Exception:
            rospy.loginfo("get_Posture error")

    def joints_limit(self, joints):
        # 创建消息包
        request = kinemaricsRequest()
        request.cur_joint1 = joints[0]
        request.cur_joint2 = joints[1]
        request.cur_joint3 = joints[2]
        request.cur_joint4 = joints[3]
        request.cur_joint5 = joints[4]
        request.kin_name = "fk"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):
                # print('response.y : ', response.y)
                # print('response.z : ', response.z)
                if 0.22 < response.z < 0.23 and response.y > 0 and -1.6 < response.Roll < -1.5:
                    move_joint1 = [90, joints[1], joints[2], joints[3], 0, 180]
                    move_joint2 = [90, joints[1], joints[2], joints[3], 0, 0]
                    if self.move_num == 1:
                        self.move_num = 2
                        self.sbus.Arm_serial_servo_write6_array(move_joint1, self.move_time)
                    if self.move_num == 2:
                        self.move_num = 1
                        self.sbus.Arm_serial_servo_write6_array(move_joint2, self.move_time)
        except Exception:
            rospy.loginfo("joints_limit error")

    def snake_run(self, point_y):
        '''
        发布位置请求,获取关节旋转角度
        '''
        request = kinemaricsRequest()
        request.tar_x = self.Posture[0]
        request.tar_y = point_y
        request.tar_z = 0.225476
        request.kin_name = "ik"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):
                # 获得反解响应结果
                joints = [0.0, 0.0, 0.0, 0.0, 0.0]
                joints[0] = response.joint1
                joints[1] = response.joint2
                joints[2] = response.joint3
                joints[3] = response.joint4
                joints[4] = response.joint5
                for i in range(1, 6):
                    if joints[i - 1] < 0: return
                self.joints_limit(joints)
        except Exception:
            rospy.loginfo("snake_run error")

    def snake_main(self, name, msg):
        self.get_Posture()
        for key, area in msg.items():
            if key == name:
                # 估计方块据摄像头的位置
                distance = 27.05 * math.pow(area, -0.51) - 0.2
                # 估计方块在世界坐标系下的位置
                target_dist = distance + self.Posture[1]
                if self.grap_status == 'Waiting':
                    threading.Thread(target=self.snake_run, args=(target_dist,)).start()
                    if self.Posture[1] <= 0.02:
                        # 摇头
                        self.sbus.Arm_serial_servo_write(5,180,300)
                        sleep(0.1)
                        self.sbus.Arm_serial_servo_write(5,0, 300)
                        sleep(0.1)
                        self.num = 1
                    elif self.Posture[1] >= 0.19:
                        # 夹爪张合
                        self.sbus.Arm_serial_servo_write(6, 0, 100)
                        sleep(0.08)
                        self.sbus.Arm_serial_servo_write(6, 180, 100)
                        sleep(0.08)
                        self.num += 1
                    else:
                        # 夹爪张合
                        self.sbus.Arm_serial_servo_write(6, 0, 100)
                        sleep(0.08)
                        self.sbus.Arm_serial_servo_write(6, 180, 100)
                        sleep(0.08)
                        self.num = 1
                    if self.num % 10 == 0: self.grap_status = 'Graping'
                elif self.grap_status == 'Graping':
                    self.grap_status = 'Runing'
                    # 执行放下
                    self.arm_move.snake_run(name)
                    # 动作完毕
                    self.num = 1
                    # 设置移动状态
                    self.grap_status = 'Waiting'
            else:
                if self.grap_status == 'Waiting':
                    # 摇头
                    self.sbus.Arm_serial_servo_write(5,180,300)
                    sleep(0.1)
                    self.sbus.Arm_serial_servo_write(5,0, 300)
                    sleep(0.1)
                    if self.Posture[1] > 0.01:
                        joint_1 = [90, 156, 16, 8, 0, 180]
                        self.sbus.Arm_serial_servo_write6_array(joint_1, self.move_time)
