#!/usr/bin/env python
# coding: utf-8
import rospy
import Arm_Lib
import cv2 as cv
from math import pi
from time import sleep
from stacking_move import stacking_move
from arm_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse


class color_stacking:
    def __init__(self):
        # 初始化一些参数
        self.image = None
        self.color_name = None
        # 机械臂识别位置调节
        self.xy=[90,135]
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        # 创建抓取实例
        self.stacking_move = stacking_move()
        # ROS节点初始化
        #self.n = rospy.init_node('ros_arm', anonymous=True)
        # 创建获取反解结果的客户端
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)

    def stacking_grap(self, msg, xy=None):
        '''
        抓取函数
        :param msg: [颜色,位置]
        '''
        if xy!=None: self.xy=xy
        for index, pos in enumerate(msg):
            try:
                # 此处ROS反解通讯,获取各关节旋转角度
                joints = self.server_joint(pos)
                num = index + 1
                # 驱动机械臂抓取
                self.stacking_move.arm_run(str(num), joints)
            except Exception:
                print("sqaure_pos empty")
            # 返回至中心位置
        self.arm.Arm_serial_servo_write(1, 90, 1000)
        sleep(1)
        # 初始位置
        joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 0]
        # 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
        sleep(1)

    def select_color(self, image, color_hsv, color_list):
        '''
        选择识别颜色
        :param image:输入图像
        :param color_list: 颜色序列:['0'：无 '1'：红色 '2'：绿色 '3'：蓝色 '4'：黄色]
        :return: 输出处理后的图像,(颜色,位置)
        '''
        # 规范输入图像大小
        self.image = cv.resize(image, (640, 480))
        msg = []
        # 获取第一个颜色序列的信息
        if len(color_list) >0:
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

    def get_Sqaure(self, hsv_lu):
        '''
        颜色识别
        :param lowerb:包含下边界数组或标量。
        :param upperb:包含上边界数组或标量。
        :return: 方块中心位置
        '''
        (lowerb, upperb) = hsv_lu
        # 复制原始图像,避免处理过程中干扰
        mask = self.image.copy()
        # 将图像转换为HSV。
        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        # 筛选出位于两个数组之间的元素。
        img = cv.inRange(hsv_img, lowerb, upperb)
        # 设置非掩码检测部分全为黑色
        mask[img == 0] = [0, 0, 0]
        # 形态学变换去出细小的干扰因素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # 将图像转为灰度图
        dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)
        # 图像二值化
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标) python2和python3在此处略有不同
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # 获取轮廓点集(坐标)
        for i, cnt in enumerate(contours):
            # boundingRect函数计算边框值，x，y是坐标值，w，h是矩形的宽和高
            x, y, w, h = cv.boundingRect(cnt)
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            # ⾯积范围
            if area > 1000:
                # 在img图像画出矩形，(x, y), (x + w, y + h)是矩形坐标，(0, 255, 0)设置通道颜色，2是设置线条粗度
                cv.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # 绘制矩形中心
                cv.circle(self.image, (int(x + w / 2), int(y + h / 2)), 5, (0, 0, 255), -1)
                # 在图片中绘制结果
                cv.putText(self.image, self.color_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                # 中心坐标
                point_x = float(x + w / 2)
                point_y = float(y + h / 2)
                # 计算方块在图像中的位置
                #(a, b) = (round(((point_x - 320) / 4000), 5), round(((480 - point_y) / 3000 + 0.2)*0.86+0.016, 5))
                (a, b) = (round(((point_x - 320) / 4000), 5), round(((240 - point_y) / 3000 + 0.265)*0.95, 5))
#                 print("------------------ stacking up -------------------")
#                 print(a, b)
#                 print("------------------ stacking down -------------------")
                return (a, b)

    def color_(self, color_hsv,name):
        '''
        获取所选颜色的位置
        :param color_hsv: 对应颜色的HSV值
        :param name: 选择颜色
        :return: (颜色,位置)
        '''
        sqaure_pos = None
        if name == "1":
            self.color_name = "red"
            # print "选择red"
            sqaure_pos = self.get_Sqaure(color_hsv["red"])
        elif name == "2":
            self.color_name = "green"
            # print "选择green"
            sqaure_pos = self.get_Sqaure(color_hsv["green"])
        elif name == "3":
            self.color_name = "blue"
            # print "选择blue"
            sqaure_pos = self.get_Sqaure(color_hsv["blue"])
        elif name == "4":
            self.color_name = "yellow"
            # print "选择yellow"
            sqaure_pos = self.get_Sqaure(color_hsv["yellow"])
        return sqaure_pos

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
                # 获取反解的响应结果
                joints = [0.0, 0.0, 0.0, 0.0, 0.0]
                joints[0] = response.joint1
                joints[1] = response.joint2
                joints[2] = response.joint3
                joints[3] = response.joint4
                joints[4] = response.joint5
#                 # 角度调整
#                 if joints[2] < 0:
#                     joints[1] += joints[2] / 2
#                     joints[3] += joints[2] * 3 / 4
#                     joints[2] = 0
                # 当逆解越界,出现负值时,适当调节.
                if joints[2] < 0:
                    joints[1] += joints[2] * 3 / 5
                    joints[3] += joints[2] * 3 / 5
                    joints[2] = 0
                # print joints
                return joints
        except Exception:
            rospy.loginfo("arg error")
