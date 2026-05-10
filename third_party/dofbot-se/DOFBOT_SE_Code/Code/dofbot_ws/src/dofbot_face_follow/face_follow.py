# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
import PID
import Arm_Lib


class face_follow:
    def __init__(self):
        self.Arm = Arm_Lib.Arm_Device()
        self.target_servox=90
        self.target_servoy=45
        self.a = 0
        self.b = 0
        self.xservo_pid = PID.PositionalPID(0.25, 0.1, 0.05)
        self.yservo_pid = PID.PositionalPID(0.25, 0.1, 0.05)
        # Create an instance of OpenCV's joint classifier
        # 创建opencv的联级分类器的实例
        self.faceDetect = cv.CascadeClassifier("haarcascade_frontalface_default.xml")

    def face_filter(self, faces):
        '''
        Filter the face
        对人脸进行一个过滤
        '''
        if len(faces) == 0: return None
        # At present, we are looking for the face with the largest area in the pictur
        # 目前找的是画面中面积最大的人脸
        max_face = max(faces, key=lambda face: face[2] * face[3])
        (x, y, w, h) = max_face
        # Set the minimum threshold of face detection
        # 设置人脸检测最小阈值
        if w < 10 or h < 10: return None
        return max_face

    def follow_function(self, img):
        self.a = 0
        self.b = 0
        img = cv.resize(img, (640, 480))
        # Copy the original image to avoid interference during processing
        # 复制原始图像,避免处理过程中干扰
        img = img.copy()
        # Convert image to grayscale
        # 将图像转为灰度图
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Face detection
        # 检测人脸
        faces = self.faceDetect.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
        if len(faces) != 0:
            face = self.face_filter(faces)
            # Face filtering
            # 人脸过滤
            (x, y, w, h) = face
            # Draw a rectangle on the original color map
            # 在原彩图上绘制矩形
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 4)
            cv.putText(img, 'Person', (280, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (105, 105, 105), 2)
            point_x = x + w / 2
            point_y = y + h / 2

            # cv.rectangle(img, (320-60, 240-60), (320-20+80, 240-20+80), (0, 255, 0), 4)
            
            if (x > 260 and x < 380 ) and (y > 180 and  y < 300):
                    self.b = 1
                    self.a = 1
            # print(self.b )
            # print(self.a )
            if not (self.target_servox>=180 and point_x<=320 and self.a == 1 or self.target_servox<=0 and point_x>=320 and self.a == 1):
                if(self.a == 0):
                    self.xservo_pid.SystemOutput = point_x
                    self.xservo_pid.SetStepSignal(320)
                    self.xservo_pid.SetInertiaTime(0.01, 0.1)
                    # print(self.yservo_pid.SystemOutput)
                    target_valuex = int(1500 + self.xservo_pid.SystemOutput)
                    self.target_servox = int((target_valuex - 500) / 10)
                  
                    # Set movement restrictions
                    # 设置移动限制
                    
                    # print("x= ",self.target_servox)
                    # print("x= ",point_x)
                    if self.target_servox > 180:self.target_servox = 180
                    if self.target_servox < 0: self.target_servox = 0
                    
            if not (self.target_servoy>=180 and point_y<=240 and self.b == 1 or self.target_servoy<=0 and point_y>=240 and self.b == 1):
                # Input Y-axis direction parameter PID control input
                # 输入Y轴方向参数PID控制输入
                if(self.b == 0):
                    
                    self.yservo_pid.SystemOutput = point_y
                    self.yservo_pid.SetStepSignal(240)
                    self.yservo_pid.SetInertiaTime(0.01, 0.1)
                    
                    target_valuey = int(1500 + self.yservo_pid.SystemOutput)
                    self.target_servoy = int((target_valuey - 500) / 10) - 45
                    # Set movement restrictions
                    # 设置移动限制
                    # print("y= ",self.target_servoy)
                    # print("y= ",point_y)
                   
                    if self.target_servoy > 360: self.target_servoy = 360
                    if self.target_servoy < 0: self.target_servoy = 0
 
            joints_0 = [self.target_servox/1, 135, self.target_servoy / 2, self.target_servoy / 2, 90, 30]
            self.Arm.Arm_serial_servo_write6_array(joints_0, 500)
        return img
