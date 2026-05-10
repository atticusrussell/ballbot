#!/usr/bin/env python
# coding: utf-8
import rospy
import Arm_Lib
import cv2 as cv
from time import sleep
from grap_move import grap_move
from arm_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse


class color_identify:
    def __init__(self):
        '''
        设置初始化参数
        '''
        self.image = None
        self.color_name = None
        # 机械臂识别位置调节
        self.xy = [90, 135]
        # 创建抓取实例
        self.grap_move = grap_move()
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        # 创建节点句柄
        self.n = rospy.init_node('ros_arm', anonymous=True)
        # 创建用于调用的ROS服务的句柄。
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)

    def select_color(self, image, color_hsv, color_list):
        '''
        选择识别颜色
        :param image:输入图像
        :param color_hsv: HSV的范围阈值
        :param color_list: 颜色序列:['0'：无 '1'：红色 '2'：绿色 '3'：蓝色 '4'：黄色]
        :return: 输出处理后的图像,(颜色,位置)
        '''
        # 规范输入图像大小
        self.image = cv.resize(image, (640, 480))
        msg = []
        # 获取第一个颜色序列的信息
        if len(color_list) > 0:
            name_pos = self.color_(color_hsv, color_list[0])
            if name_pos != None: msg.append(name_pos)
        # 获取第二个颜色序列的信息
        if len(color_list) > 1:
            name_pos = self.color_(color_hsv, color_list[1])
            if name_pos != None: msg.append(name_pos)
        # 获取第三个颜色序列的信息
        if len(color_list) > 2:
            name_pos = self.color_(color_hsv, color_list[2])
            if name_pos != None: msg.append(name_pos)
        # 获取第四个颜色序列的信息
        if len(color_list) > 3:
            name_pos = self.color_(color_hsv, color_list[3])
            if name_pos != None: msg.append(name_pos)
        return self.image, msg

    def color_(self, color_hsv, name):
        '''
        获取所选颜色的位置
        :param color_hsv: 对应颜色的HSV值
        :param name: 选择颜色
        :return: (颜色,位置)
        '''
        # 初始化msg元组
        msg = None
        if name == "1":
            self.color_name = "red"
            # print "选择red"
            sqaure_pos = self.get_Sqaure(color_hsv["red"])
            msg = ("red", sqaure_pos)
        elif name == "2":
            self.color_name = "green"
            # print "选择green"
            sqaure_pos = self.get_Sqaure(color_hsv["green"])
            msg = ("green", sqaure_pos)
        elif name == "3":
            self.color_name = "blue"
            # print "选择blue"
            sqaure_pos = self.get_Sqaure(color_hsv["blue"])
            msg = ("blue", sqaure_pos)
        elif name == "4":
            self.color_name = "yellow"
            # print "选择yellow"
            sqaure_pos = self.get_Sqaure(color_hsv["yellow"])
            msg = ("yellow", sqaure_pos)
        return msg

    def get_Sqaure(self, hsv_lu):
        '''
        颜色识别
        :param hsv_lu:(lowerb, upperb)
        :return: 方块中心位置
        '''
        (lowerb, upperb) = hsv_lu
        # 复制原始图像,避免处理过程中干扰
        mask = self.image.copy()
        # 将图像转换为HSV。
        HSV_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        # 筛选出位于两个数组之间的元素。
        img = cv.inRange(HSV_img, lowerb, upperb)
        # 设置非掩码检测部分全为黑色
        mask[img == 0] = [0, 0, 0]
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # 将图像转为灰度图
        dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标) python2和python3在此处略有不同
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # 获取轮廓点集(坐标)
        for i, cnt in enumerate(contours):
            # boundingRect函数计算边框值，x，y是坐标值，w，h是矩形的宽和高
            x, y, w, h = cv.boundingRect(cnt)
            # 中心坐标
            point_x = float(x + w / 2)
            point_y = float(y + h / 2)
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            # ⾯积范围
            if area > 1000:
                # 在img图像画出矩形，(x, y), (x + w, y + h)是矩形坐标，(0, 255, 0)设置通道颜色，2是设置线条粗度
                cv.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # 绘制矩形中心
                cv.circle(self.image, (int(point_x), int(point_y)), 5, (0, 0, 255), -1)
                # # 在图片中绘制结果
                cv.putText(self.image, self.color_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                # # 计算方块在图像中的位置
                # (a, b) = (round(((point_x - 320) / 4000), 5), round(((480 - point_y) / 3000 + 0.2) * 0.86 + 0.016, 5))
                (a, b) = (round(((point_x - 320) / 4000), 5), round(((240 - point_y) / 3000 + 0.265)*0.95, 5))
#                 print("------------------ identify up -------------------")
#                 print(a, b)
#                 print("------------------ identify down -------------------")
                return (a, b)

    def identify_grap(self, msg, xy=None):
        '''
        抓取函数
        :param msg: (颜色,位置)
        '''
        if xy != None: self.xy = xy
        for name, pos in msg:
#             print ("pos : ",pos)
#             print ("name : ",name)
            try:
                # 此处ROS反解通讯,获取各关节旋转角度
                joints = self.server_joint(pos)
                # 调取移动函数
                self.grap_move.arm_run(str(name), joints)
            except Exception:
                print("sqaure_pos empty")
        move_status=0
        for i in range(len(msg)):
            if msg[i][1]!=None:move_status=1
        if move_status==1:
            # 架起
            joints_uu = [90, 80, 50, 50, 265, 0]
            # 移动至物体位置上方
            self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
            sleep(1)
            # 初始位置
            joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 0]
            # 移动至初始位置
            self.arm.Arm_serial_servo_write6_array(joints_0, 500)
            sleep(0.5)

    def server_joint(self, posxy):
        '''
        发布位置请求,获取关节旋转角度
        :param posxy: 位置点x,y坐标
        :return: 每个关节旋转角度
        '''
        # 等待server端启动
        self.client.wait_for_service()
        # 创建消息包
        request = kinemaricsRequest()
        request.tar_x = posxy[0]
        request.tar_y = posxy[1]
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
                # 当逆解越界,出现负值时,适当调节.
#                 if -10<joints[2] < 0:
#                     joints[2] = 0
                if joints[2] <= 0:
                    joints[1] += joints[2] * 3 / 5
                    joints[3] += joints[2] * 3 / 5
                    joints[2] = 0
#                 print (joints)
                return joints
        except Exception:
            rospy.loginfo("arg error")
