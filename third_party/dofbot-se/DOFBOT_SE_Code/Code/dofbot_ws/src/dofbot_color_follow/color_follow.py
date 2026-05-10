# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
import PID
import Arm_Lib

class color_follow:
    def __init__(self):
        self.img = None
        self.target_servox=90
        self.target_servoy=45
        self.Arm = Arm_Lib.Arm_Device()
        self.xservo_pid = PID.PositionalPID(0.5, 0.1, 0.05)
        self.yservo_pid = PID.PositionalPID(0.3, 0.1, 0.05)
        self.b = 0
        self.a = 0
    def follow_function(self, img, HSV_config):
        self.b = 0
        self.a = 0
        (color_lower, color_upper) = HSV_config
        self.img = cv.resize(img, (640, 480), )
        self.img = cv.GaussianBlur(self.img, (5, 5), 0)
        hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, color_lower, color_upper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        mask = cv.GaussianBlur(mask, (5, 5), 0)
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        
        if len(cnts) > 0:
            cnt = max(cnts, key=cv.contourArea)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(cnt)
            if color_radius > 10:
                # Mark the detected color with the prototype coil
                # 将检测到的颜色用原形线圈标记出来
                cv.circle(self.img, (int(color_x), int(color_y)), int(color_radius), (255, 0, 255), 3)
                # a= color_x+color_radius
                # b= color_y+color_radius
                # self.xservo_pid.SystemOutput = color_x
                # self.xservo_pid.SetStepSignal(320)
                # self.xservo_pid.SetInertiaTime(0.01, 0.1)
                
                # cv.rectangle(self.img, (320-80, 240-80), (320-20+100, 240-20+100), (0, 255, 0), 4)
            
                if (color_x > 230 and color_x < 410 ) and (color_y > 130 and  color_y < 330):
                    self.b = 1
                    self.a = 1
                    
                if not (self.target_servox>=180 and color_x<320 and self.a ==1 or self.target_servox<=0 and color_x>320 and self.a == 1):
                    if(self.a == 0):
                        self.xservo_pid.SystemOutput = color_x
                        self.xservo_pid.SetStepSignal(320)
                        self.xservo_pid.SetInertiaTime(0.01, 0.1)
                        target_valuex = int(1500 + self.xservo_pid.SystemOutput)
                        self.target_servox = int((target_valuex - 500) / 10) 
                        
                        # print(self.target_servox)
                        # Set movement restrictions
                        # 设置移动限制
                        
                        # if (self.target_servox > 85 and self.target_servox < 115) :self.target_servox = 100
                            
                        if self.target_servox > 180:self.target_servox = 180
                        if self.target_servox < 0: self.target_servox = 0
                        
                if not (self.target_servoy>=180 and color_y<=240 and self.b == 1 or self.target_servoy<=0 and color_y>=240 and self.b == 1):
                    # Input Y axis direction parameter PID control input
                    # 输入Y轴方向参数PID控制输入
                    if(self.b == 0):
                        self.yservo_pid.SystemOutput = color_y
                        self.yservo_pid.SetStepSignal(240)
                        self.yservo_pid.SetInertiaTime(0.01, 0.1)
                        target_valuey = int(1500 + self.yservo_pid.SystemOutput)
                        self.target_servoy = int((target_valuey - 500) / 10) - 45
                        # a = target_valuey
                        # print(self.target_servoy)
                        # Set movement restrictions
                        # 设置移动限制
    
                        # if (self.target_servox > 45 and self.target_servox < 65) :self.target_servox = 55
                        if self.target_servoy > 360: self.target_servoy = 360
                        if self.target_servoy < 0: self.target_servoy = 0
                
                joints_0 = [self.target_servox, 135, self.target_servoy / 2, self.target_servoy / 2, 90, 30]
                self.Arm.Arm_serial_servo_write6_array(joints_0, 500)
        return self.img

    def get_hsv(self, img):
        '''
        Get the range of HSV in a region
        获取某一区域的HSV的范围
        :param img: 彩色图    color image
        :return: 图像和HSV的范围   Image and HSV range
        '''
        print("okok")
        H = [];S = [];V = []
        img = cv.resize(img, (640, 480), )
        # Convert color image to HSV
        # 将彩色图转成HSV
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # draw a rectangle
        # 画矩形框
        point_init_x=290
        point_init_y=300
        point_end_x=350
        point_end_y=360
        cv.rectangle(img, (point_init_x, point_init_y), (point_end_x, point_end_y), (0, 255, 0), 2)
        # Take out the H, S, V values ​​of each row and column in turn and put them into the container
        # 依次取出每行每列的H,S,V值放入容器中
        for i in range(point_init_x, point_end_x):
            for j in range(point_init_y, point_end_y):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        # Calculate the maximum and minimum of H, S, and V respectively
        # 分别计算出H,S,V的最大最小
        H_min = min(H);H_max = max(H)
        S_min = min(S);S_max = max(S)
        V_min = min(V);V_max = max(V)
        # HSV range adjustment
        # HSV范围调整
        if H_min - 2 < 0:H_min = 0
        else:H_min -= 2
        if S_min - 15 < 0:S_min = 0
        else:S_min -= 15
        if V_min - 15 < 0:V_min = 0
        else:V_min -= 15
        if H_max + 2 > 255:H_max = 255
        else:H_max += 2
        S_max = 255;V_max = 255
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        txt1 = 'Learning ...'
        txt2 = 'OK !!!'
        if S_min < 5 or V_min < 5:cv.putText(img, txt1, (230, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:cv.putText(img, txt2, (270, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv.putText(img, lowerb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv.putText(img, upperb, (150, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range

    def learning_follow(self, img, HSV_config):
        img = self.follow_function(img, HSV_config)
        return img
