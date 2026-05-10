#!/usr/bin/env python
# coding: utf-8
import threading
import cv2 as cv
from garbage_identify import garbage_identify
from single_garbage_move import single_garbage_move


class single_garbage_identify:
    def __init__(self):
        # 创建机械臂实例
        self.garbage_index = 0
        self.name_tmp = ' '
        self.garbage_num = 'None'
        self.garbage_class = 'None'
        # 初始化计数器
        self.num = 0
        # 初始化运动状态
        self.status = 'waiting'
        self.garbage_identify = garbage_identify()
        self.single_garbage_move = single_garbage_move()

    def single_garbage_run(self, image):
        '''
        执行垃圾识别函数
        :param image: 原始图像
        :return: 识别后的图像,识别信息(name, pos)
        '''
        # 规范输入图像大小
        self.frame = cv.resize(image, (640, 480))
        try: self.garbage_getName()
        except Exception: print("sqaure_pos empty")
        return self.frame,self.garbage_num, self.garbage_class

    def garbage_getName(self):
        name = "None"
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
        if self.name_tmp == name and self.name_tmp != 'None':
            self.num += 1
            # 每当连续识别20次并且运动状态为waiting的情况下,执行抓取任务
            if self.num % 10 == 0 and self.status == 'waiting':
                self.status = 'Runing'
                # 开启抓取线程
                threading.Thread(target=self.single_garbage_move.single_garbage_grap, args=(self.garbage_class,)).start()
                self.num = 0
                self.status = 'waiting'
        else: self.name_tmp = name
