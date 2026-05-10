# !/usr/bin/env python
# coding: utf-8
import math
import rospy
import threading
import Arm_Lib
from time import sleep
from snake_move import snake_move
from dofbot_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse


class snake_ctrl:
    def __init__(self):
        self.sbus = Arm_Lib.Arm_Device()
        self.arm_move = snake_move()
        self.color_name = None
        self.image = None
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_time = 600
        # 设置移动状态 set mobile state
        self.grap_status = 'Waiting'
        # 夹爪加紧角度 Gripper tightening angle
        self.grap_joint = 145
        self.num = 1
        self.move_num = 1
        self.n = rospy.init_node('dofbot_snake', anonymous=True)

        # 创建用于调用的ROS服务的句柄 Create a handle to the ROS service to invoke
        self.client = rospy.ServiceProxy("dofbot_kinemarics", kinemarics)
	
    def read_joint(self):
        '''
        Loop to read the current angle of the servo
        循环读取舵机的当前角度
        '''
        num = 0
        # sleep(1)
        for i in range(1, 6):
            # while 1:
                joint = self.sbus.Arm_serial_servo_read(i)
                # sleep(3)
                if joint != None:
                    self.cur_joint[i - 1] = joint
                    break
                num += 1
                #print("Current joint angle : {}".format(self.cur_joint))

    def get_Posture(self,a,b,c,d,e):
    # def get_Posture(self):
        '''
        Publish joint angle, get position
        发布关节角度,获取位置
        '''
        # self.read_joint()
        # sleep(1)
        # 等待server端启动 Wait for the server to start
        self.client.wait_for_service()
        # 创建消息包 Create a message pack
        request = kinemaricsRequest()
        request.cur_joint1 = a
        request.cur_joint2 = b
        request.cur_joint3 = c
        request.cur_joint4 = d
        request.cur_joint5 = e
        request.kin_name = "fk"

        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):
                # 获得反解响应结果 Get the inverse solution response result
                self.Posture[0] = response.x
                self.Posture[1] = response.y
                self.Posture[2] = response.z
                self.Posture[3] = response.Roll
                self.Posture[4] = response.Pitch
                self.Posture[5] = response.Yaw
                
        except Exception:
            rospy.loginfo("get_Posture error")

    def joints_limit(self, joints):
        # 创建消息包 Create a message pack
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
                
             #   print('response.y : ', response.y)
                #print('response.z : ', response.z)
                #print(request.cur_joint1,request.cur_joint2,request.cur_joint3)
                if 0.22 < response.z < 0.23 and response.y > 0 and -1.6 < response.Roll < -1.5:
                    move_joint1 = [90, joints[1], joints[2], joints[3], 0, 180]                    
                    move_joint2 = [90, joints[1], joints[2], joints[3], 0, 30]
                    if self.move_num == 1:
                        self.sbus.Arm_serial_servo_write6_array(move_joint1, self.move_time)
                        self.move_num = 2
                    if self.move_num == 2:
                        self.sbus.Arm_serial_servo_write6_array(move_joint2, self.move_time)
                        self.move_num = 1
        except Exception:
            rospy.loginfo("joints_limit error")

    def snake_run(self, point_y):
        '''
        Post position request, get joint rotation angle
        发布位置请求,获取关节旋转角度
        '''
        # next = self.sbus.Arm_serial_servo_read(1)
        # self.Posture[0] = next
        # sleep(.01)
        # for i in range(5):
        #     joint = self.sbus.Arm_serial_servo_read(i+1)
        #     self.cur_joint[i] = joint
        
        request = kinemaricsRequest()
        request.tar_x = self.Posture[0]
        request.tar_y = point_y
        request.tar_z = 0.225476
        request.kin_name = "ik"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):
                # Get the inverse solution response result
                # 获得反解响应结果
                joints = [0.0, 0.0, 0.0, 0.0, 0.0]
                joints[0] = response.joint1
                joints[1] = response.joint2
                joints[2] = response.joint3
                joints[3] = response.joint4
                joints[4] = response.joint5
                # for i in range(1, 6):
                #     if joints[i - 1] < 0: return
                self.joints_limit(joints)
        except Exception:
            rospy.loginfo("snake_run error")

    def snake_main(self, name, msg):
    # def snake_main(self, name, msg):
        for i in range(5):
            joint = self.sbus.Arm_serial_servo_read(i+1)
            self.cur_joint[i] = joint
         
        s1,s2, s3, s4,s5 = self.cur_joint[0], self.cur_joint[1], self.cur_joint[2],self.cur_joint[3],self.cur_joint[4]
        self.get_Posture(s1,s2,s3,s4,s5)
        # self.get_Posture()
        # sleep(5)
        # print(self.Posture[1])
             
        for key, area in msg.items():
            if key == name:
                
                # Estimate the position of the block according to the camera
                # 估计方块据摄像头的位置
                distance = 27.03 * math.pow(area, -0.51) - 0.2
                # Estimate the position of the block in the world coordinate system
                # 估计方块在世界坐标系下的位置
                target_dist = distance + self.Posture[1]
                # print(target_dist)
   
                if self.grap_status == 'Waiting':
                    threading.Thread(target=self.snake_run, args=(target_dist,)).start()
                    
                    print(self.Posture[1])
                    if self.Posture[1] <= 0.02:
                        # shaking his head
                        # 摇头                      
                        self.sbus.Arm_serial_servo_write(5,180,300)
                        sleep(0.08)
                        self.sbus.Arm_serial_servo_write(5,0, 300)
                        sleep(0.1)

                        self.num = 1
                        
                    elif self.Posture[1] >= 0.19:
                        # Gripper opening and closing
                        # 夹爪张合
                        self.sbus.Arm_serial_servo_write(6, 0, 100)
                        sleep(0.08)
                        self.sbus.Arm_serial_servo_write(6, 180, 100)
                        sleep(0.08)
                        self.num += 1
                    else:
                        # Gripper opening and closing
                        # 夹爪张合
                        self.sbus.Arm_serial_servo_write(6, 0, 100)
                        sleep(0.08)
                        self.sbus.Arm_serial_servo_write(6, 180, 100)
                        sleep(0.08)
                        self.num = 1
                    if self.num % 5 == 0: self.grap_status = 'Graping'
                        
                elif self.grap_status == 'Graping':
                    self.grap_status = 'Runing'
                    # 执行放下 put down
                    self.arm_move.snake_run(name)
                    sleep(.001)
                    # 动作完毕 action completed
                    self.num = 1
                    # 设置移动状态 set mobile state
                    self.grap_status = 'Waiting'
            else:
                if self.grap_status == 'Waiting':
                    # 摇头 shaking his head
                  
                    self.sbus.Arm_serial_servo_write(5,180,300)
                    sleep(0.08)
                    self.sbus.Arm_serial_servo_write(5,0, 300)
                    sleep(0.08)
                    if self.Posture[1] > 0.01:
                        joint_1 = [90, 156, 16, 8, 0, 180]
                        self.sbus.Arm_serial_servo_write6_array(joint_1, self.move_time)

