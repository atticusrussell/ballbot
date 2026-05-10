#!/usr/bin/env python
# coding: utf-8
import rospy
import Arm_Lib
import cv2 as cv
from math import pi
from time import sleep
from stacking_grap import stacking_grap
from dofbot_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse


class stacking_GetTarget:
    def __init__(self):
        self.image = None
        self.color_name = None
        self.color_status = True
        # Initial position of manipulator
        # 机械臂初始位置
        self.xy = [90, 135]
        self.arm = Arm_Lib.Arm_Device()
        self.grap = stacking_grap()
        self.n = rospy.init_node('dofbot_stacking', anonymous=True)
        # Create a client to get the inverse solution result
        # 创建获取反解结果的客户端
        self.client = rospy.ServiceProxy("dofbot_kinemarics", kinemarics)

    def target_run(self, msg, xy=None):
        '''
        Grab function 抓取函数
        :param msg: [颜色,位置] [color, location]
        '''
        if xy != None: self.xy = xy
        num = 1
        move_status=0
        for i in msg.values():
            if i !=None: move_status=1
        if move_status==1:
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)
        for name, pos in msg.items():
            try:
                joints = self.server_joint(pos)
                self.grap.arm_run(str(num), joints)
                num += 1
            except Exception:
                print("sqaure_pos empty")
        # Return to center position
        # 返回至中心位置
        self.arm.Arm_serial_servo_write(1, 90, 1000)
        sleep(1)
        # initial position
        # 初始位置
        joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]
        # Move to initial position
        # 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
        sleep(1)

    def select_color(self, image, color_hsv, color_list):
        '''
        Select recognition color 选择识别颜色
        :param image:输入图像 input image
        :param color_list: 颜色序列:['0'：无 '1'：红色 '2'：绿色 '3'：蓝色 '4'：黄色] Color sequence: ['0': none '1': Red '2': Green '3': Blue '4': yellow]
        :return: 输出处理后的图像,(颜色,位置) Output the processed image, (color, position)
        '''
        # 规范输入图像大小 Normalized input image size
        self.image = cv.resize(image, (640, 480))
        msg = {}
        if len(color_list) == 0: return self.image, msg
        if '4' in color_list:
            self.color_name = color_list['4']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        if '3' in color_list:
            self.color_name = color_list['3']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        if '2' in color_list:
            self.color_name = color_list['2']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        if '1' in color_list:
            self.color_name = color_list['1']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos != None: msg[self.color_name] = pos
        return self.image, msg

    def get_Sqaure(self, hsv_lu):
        '''
        颜色识别  color recognition
        :param hsv_lu:(lowerb, upperb)
        :return: 方块中心位置  block center position
        '''
        (lowerb, upperb) = hsv_lu
        # Copy the original image to avoid interference during processing
        # 复制原始图像,避免处理过程中干扰
        mask = self.image.copy()
        # Convert image to HSV
        # 将图像转换为HSV
        HSV_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        # filter out elements between two arrays
        # 筛选出位于两个数组之间的元素
        img = cv.inRange(HSV_img, lowerb, upperb)
        # Set the non-mask detection part to be all black
        # 设置非掩码检测部分全为黑色
        mask[img == 0] = [0, 0, 0]
        # Get structuring elements of different shapes
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # morphological closure
        # 形态学闭操作
        dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # Convert image to grayscale
        # 将图像转为灰度图
        dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)
        # Image Binarization Operation
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # Get the set of contour points (coordinates)
        # 获取轮廓点集(坐标)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3: contours = find_contours[1]
        else: contours = find_contours[0]
        for i, cnt in enumerate(contours):
            x, y, w, h = cv.boundingRect(cnt)
            area = cv.contourArea(cnt)
            if area > 1000:
                # 中心坐标
                point_x = float(x + w / 2)
                point_y = float(y + h / 2)
                cv.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.circle(self.image, (int(point_x), int(point_y)), 5, (0, 0, 255), -1)
                cv.putText(self.image, self.color_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                # Calculate the position of the block in the image
                # 计算方块在图像中的位置
                (a, b) = (round(((point_x - 320) / 4000), 5), round(((480 - point_y) / 3000) * 0.8+0.19, 5))
                return (a, b)

    def server_joint(self, posxy):
        '''
        Post position request, get joint rotation angle
        发布位置请求,获取关节旋转角度
        :param posxy: 位置点x,y坐标 Location point x,y coordinates
        :return: 每个关节旋转角度    Rotation angle of each joint
        '''
        # Wait for the server to start
        # 等待server端启动
        self.client.wait_for_service()
        # Create a message pack
        # 创建消息包
        request = kinemaricsRequest()
        request.tar_x = posxy[0]
        request.tar_y = posxy[1]
        request.kin_name = "ik"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):
                # Here, ROS inversely solves the communication to obtain the rotation angle of each joint
                # 此处ROS反解通讯,获取各关节旋转角度
                joints = [0.0, 0.0, 0.0, 0.0, 0.0]
                joints[0] = response.joint1
                joints[1] = response.joint2
                joints[2] = response.joint3
                joints[3] = response.joint4
                joints[4] = response.joint5
                # When the inverse solution is out of bounds and a negative value appears, adjust it appropriately.
                # 当逆解越界,出现负值时,适当调节.
                if joints[2] < 0:
                    joints[1] += joints[2] * 3 / 5
                    joints[3] += joints[2] * 3 / 5
                    joints[2] = 0
                # print joints
                return joints
        except Exception:
            rospy.loginfo("arg error")
