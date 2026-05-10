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
        while 1:
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


   