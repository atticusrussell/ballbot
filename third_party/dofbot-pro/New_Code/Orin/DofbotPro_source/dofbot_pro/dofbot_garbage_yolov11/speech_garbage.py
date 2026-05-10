#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
import threading
import cv2 as cv
import time
from time import sleep
from garbage_identify import garbage_identify


from Speech_Lib import Speech
mySpeech = Speech()

class speech_garbage:
    def __init__(self):
        # 中间变量
        self.name_tmp = ' '
        # 初始化垃圾名称
        self.garbage_num = 'None'
        # 初始化垃圾类别
        self.garbage_class = 'None'
        # 初始化计数器
        self.num = 0
        # 初始化运动状态
        self.status = 'waiting'
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device()
        # 夹爪加紧角度
        self.grap_joint = 135
        # 初始化垃圾识别实例
        self.garbage_identify = garbage_identify()


    def single_garbage_run(self, image):
        '''
        执行垃圾识别函数
        :param image: 原始图像
        :return: 识别后的图像
        '''
        self.frame = image.copy()
        self.garbage_getName()
        # 规范输入图像大小
        # self.frame = cv.resize(image, (640, 480))
        # try: self.garbage_getName()
        # except Exception: print("sqaure_pos empty")
        return self.frame
    
    def garbage_getName(self):
        name = "None"
        if self.status == 'waiting':
            self.frame, msg = self.garbage_identify.garbage_run(self.frame)
            for key, pos in msg.items(): name = key
            if name == "Zip_top_can":              (self.garbage_num, self.garbage_class) = ('00', '01')
            if name == "Old_school_bag":           (self.garbage_num, self.garbage_class) = ('01', '01')
            if name == "Newspaper":                (self.garbage_num, self.garbage_class) = ('02', '01')
            if name == "Book":                     (self.garbage_num, self.garbage_class) = ('03', '01')
            if name == "Toilet_paper":             (self.garbage_num, self.garbage_class) = ('04', '02')
            if name == "Peach_pit":                (self.garbage_num, self.garbage_class) = ('05', '02')
            if name == "Cigarette_butts":          (self.garbage_num, self.garbage_class) = ('06', '02')
            if name == "Disposable_chopsticks":    (self.garbage_num, self.garbage_class) = ('07', '02')
            if name == "Egg_shell":                (self.garbage_num, self.garbage_class) = ('08', '03')
            if name == "Apple_core":               (self.garbage_num, self.garbage_class) = ('09', '03')
            if name == "Watermelon_rind":          (self.garbage_num, self.garbage_class) = ('10', '03')
            if name == "Fish_bone":                (self.garbage_num, self.garbage_class) = ('11', '03')
            if name == "Expired_tablets":          (self.garbage_num, self.garbage_class) = ('12', '04')
            if name == "Expired_cosmetics":        (self.garbage_num, self.garbage_class) = ('13', '04')
            if name == "Used_batteries":           (self.garbage_num, self.garbage_class) = ('14', '04')
            if name == "Syringe":                  (self.garbage_num, self.garbage_class) = ('15', '04')
            if name == "None":                     (self.garbage_num, self.garbage_class) = ('None', 'None')
            if self.name_tmp == name and self.name_tmp != "None":
                self.num += 1
                # 每当连续识别3次并且运动状态为waiting的情况下,执行抓取任务
                if self.num % 3 == 0 and self.status == 'waiting':
                    self.status = 'speech'
                    self.num = 0  
                print(self.num)
            else:
                self.name_tmp = name
        elif self.status == 'speech':
            result = mySpeech.speech_read()
            if result == 94:
                if self.garbage_num == '00':
                    mySpeech.void_write(94)

                elif self.garbage_num == '01':
                    mySpeech.void_write(95)
 
                elif self.garbage_num == '02':
                    mySpeech.void_write(96)
 
                elif self.garbage_num == '03':
                    mySpeech.void_write(97)
 
                elif self.garbage_num == '04':
                    mySpeech.void_write(109)

                elif self.garbage_num == '05':
                    mySpeech.void_write(108)

                elif self.garbage_num == '06':
                    mySpeech.void_write(107)
 
                elif self.garbage_num == '07':
                    mySpeech.void_write(106)
 
                elif self.garbage_num == '08':
                    mySpeech.void_write(105)

                elif self.garbage_num == '09':
                    mySpeech.void_write(104)

                elif self.garbage_num == '10':
                    mySpeech.void_write(103)
 
                elif self.garbage_num == '11':
                    mySpeech.void_write(102)

                elif self.garbage_num == '12':
                    mySpeech.void_write(101)

                elif self.garbage_num == '13':
                    mySpeech.void_write(100)

                elif self.garbage_num == '14':
                    mySpeech.void_write(99)

                elif self.garbage_num == '15':
                    mySpeech.void_write(98)
 
                self.status = 'waiting'
                # 开启抓取线程
                #threading.Thread(target=self.single_garbage_grap, args=(self.garbage_class,)).start()

    def move(self, joints_down):
        '''
        移动过程
        :param joints_down: 机械臂抬起各关节角度
        :param color_angle: 移动到对应垃圾桶的角度
        '''
        # 初始位置
        joints_0 = [90, 90, 20, 15, 90, 30]
        # 抓取角度
        joints = [90, 40, 30, 67, 265, 30]
        joints_uu = [90, 80, 50, 50, 265, self.grap_joint]
        # 抬起
        joints_up = [joints_down[0], 80, 50, 50, 265, 30]
        # 移动至物体位置上方
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # 松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 500)
        sleep(0.5)
        # 进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # 抬起至对应位置上方
        self.arm.Arm_serial_servo_write(1, joints_down[0], 500)
        sleep(0.5)
        # 抬起至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1)
        # 释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # 抬起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)

    def single_garbage_grap(self, name):
        '''
        机械臂移动函数
        :param name:识别的垃圾类别
        '''
        self.arm.Arm_Buzzer_On(1)
        sleep(0.5)
        # 有害垃圾--红色 04
        if name == "04":
            # print("有害垃圾")
            # 移动到垃圾桶前对应姿态
            joints_down = [45, 80, 35, 40, 265, self.grap_joint]
            # 移动到垃圾桶位置放下对应姿态
            # joints_down = [45, 50, 20, 60, 265, self.grap_joint]
            # 移动
            self.move(joints_down)
            # 移动完毕
            self.status = 'waiting'
        # 可回收垃圾--蓝色 01
        if name == "01":
            # print("可回收垃圾")
            joints_down = [27, 110, 0, 40, 265, self.grap_joint]
            # joints_down = [27, 75, 0, 50, 265, self.grap_joint]
            self.move(joints_down)
            self.status = 'waiting'
        # 厨余垃圾--绿色 03
        if name == "03":
            # print("厨余垃圾")
            joints_down = [152, 110, 0, 40, 265, self.grap_joint]
            # joints_down = [147, 75, 0, 50, 265, self.grap_joint]
            self.move(joints_down)
            self.status = 'waiting'
        # 其他垃圾--灰色 02
        if name == "02":
            # print("其他垃圾")
            joints_down = [137, 80, 35, 40, 265, self.grap_joint]
            # joints_down = [133, 50, 20, 60, 265, self.grap_joint]
            self.move(joints_down)
            self.status = 'waiting'
    