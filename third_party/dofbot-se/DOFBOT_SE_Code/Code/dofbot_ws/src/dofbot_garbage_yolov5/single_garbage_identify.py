#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
import threading
import cv2 as cv
from time import sleep
from garbage_identify import garbage_identify


class single_garbage_identify:
    def __init__(self):
        # Intermediate variables
        # 中间变量
        self.name_tmp = ' '
        self.garbage_num = 'None'
        self.garbage_class = 'None'
        self.num = 0
        self.status = 'waiting'
        self.arm = Arm_Lib.Arm_Device()
        # Gripper tightening angle
        # 夹爪加紧角度
        self.grap_joint = 135
        # Initialize the garbage identification instance
        # 初始化垃圾识别实例
        self.garbage_identify = garbage_identify()

    def single_garbage_run(self, image):
        '''
        Execute the garbage identification function
        执行垃圾识别函数
        :param image: 原始图像 The original image
        :return: 识别后的图像,识别信息(name, pos) Recognized image, identification information (name, pos)
        '''
        self.frame = cv.resize(image, (640, 480))
        try: self.garbage_getName()
        except Exception: print("sqaure_pos empty")
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
                # Every time it is recognized 10 times in a row and the motion state is waiting, execute the grab task
                # 每当连续识别10次并且运动状态为waiting的情况下,执行抓取任务
                if self.num % 10 == 0 and self.status == 'waiting':
                    self.status = 'Runing'
                    # Start the crawling thread
                    # 开启抓取线程
                    threading.Thread(target=self.single_garbage_grap, args=(self.garbage_class,)).start()
                    self.num = 0
            else:
                self.name_tmp = name

    def move(self, joints_down):
        '''
        移动过程  moving process
        :param joints_down: 机械臂抬起各关节角度 The mechanical arm lifts each joint angle
        '''
        joints_0 = [90, 90, 15, 20, 90, 30]
        joints = [90, 40, 30, 72, 265, 30]
        joints_uu = [90, 80, 50, 50, 265, self.grap_joint]
        # put up 抬起
        joints_up = [joints_down[0], 80, 50, 50, 265, 30]
        # Move over the object's position 移动至物体位置上方
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # Release the jaws 松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # move to object position 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 500)
        sleep(0.5)
        # gripping, clamping jaws 进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # set up 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # Lift to the top of the corresponding position 抬起至对应位置上方
        self.arm.Arm_serial_servo_write(1, joints_down[0], 500)
        sleep(0.5)
        # Lift to the corresponding position 抬起至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1)
        # Release the object, release the gripper释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # put up 抬起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # move to initial position 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(joints_0, 1000)

    def single_garbage_grap(self, name):
        '''
        Robot arm movement function
        机械臂移动函数
        :param name:识别的垃圾类别 Identified garbage categories
        '''
        self.arm.Arm_Buzzer_On(1)
        sleep(0.5)
        # Hazardous waste - red
        # 有害垃圾--红色 04
        if name == "04":
            # print("Hazardous waste")
            # The corresponding gesture before moving to the trash can
            # 移动到垃圾桶前对应姿态
            joints_down = [45, 80, 35, 40, 265, self.grap_joint]
            # 移动到垃圾桶位置放下对应姿态
#             joints_down = [45, 50, 20, 60, 265, self.grap_joint]
            self.move(joints_down)
            self.status = 'waiting'
        # Recyclable waste - blue
        # 可回收垃圾--蓝色 01
        if name == "01":
            # print("Recyclable waste")
            joints_down = [27, 110, 0, 40, 265, self.grap_joint]
#             joints_down = [27, 75, 0, 50, 265, self.grap_joint]
            self.move(joints_down)
            self.status = 'waiting'
        # Kitchen waste - green
        # 厨余垃圾--绿色 03
        if name == "03":
            # print("Kitchen waste")
            joints_down = [152, 110, 0, 40, 265, self.grap_joint]
#             joints_down = [147, 75, 0, 50, 265, self.grap_joint]
            self.move(joints_down)
            self.status = 'waiting'
        # Other garbage--gray
        # 其他垃圾--灰色 02
        if name == "02":
            # print("Other garbage")
            joints_down = [137, 80, 35, 40, 265, self.grap_joint]
#             joints_down = [133, 50, 20, 60, 265, self.grap_joint]
            self.move(joints_down)
            self.status = 'waiting'
