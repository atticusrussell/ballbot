#!/usr/bin/env python3
# coding: utf-8
import time
import torch
import rospy
import Arm_Lib
import cv2 as cv
import numpy as np
from time import sleep
from numpy import random

from utils.torch_utils import select_device

from models.experimental import attempt_load

from utils.general import non_max_suppression, scale_coords, xyxy2xywh, plot_one_box

model_path = '/home/yahboom/Dofbot/6.AI_Visual/model0.pt'
# Initialize
device = select_device()
# Load model
model = attempt_load(model_path, map_location=device)
# Get names and colors
names = model.module.names if hasattr(model, 'module') else model.names
# Get the color value randomly
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]


class garbage_identify:
    def __init__(self):
        self.frame = None
        self.xy = [90, 130]
        self.garbage_index=0

    def garbage_run(self, image):
        '''
        执行垃圾识别函数  Execute the garbage identification function
        :param image: 原始图像     The original image
        :return: 识别后的图像,识别信息(name, pos) Recognized image, identification information (name, pos)
        '''
        self.frame = cv.resize(image, (640, 480))
        txt0 = 'Model-Loading...'
        msg={}
        if self.garbage_index<3:
            cv.putText(self.frame, txt0, (190, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            self.garbage_index+=1
            return self.frame,msg 
        if self.garbage_index>=3:
            # get identifying message
            # 获取识别消息
            try: msg = self.get_pos()
            except Exception: print("get_pos NoneType",msg)
            return self.frame, msg
            
    def get_pos(self):
        '''
        获取识别信息 Obtain identifying information
        :return: 名称,位置 name, location
        '''
        
        # Copy the original image to avoid interference during processing
        # 复制原始图像,避免处理过程中干扰
        img = self.frame.copy()
        # Reverse or arrange the axes of an array; return the modified array
        # 反转或排列数组的轴；返回修改后的数组
        img = np.transpose(img, (2, 0, 1))
        img = torch.from_numpy(img).to(device)
        
        # Data type conversion uint8 to fp16/32
        # 数据类型转换 uint8 to fp16/32
        img = img.float()
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3: img = img.unsqueeze(0)
        # Inference
        
        pred = model(img)[0]
        # Get current time
        prev_time = time.time()
        # Apply NMS
        #print(pred)
        pred = non_max_suppression(pred, 0.4, 0.5)
        
        
        gn = torch.tensor(self.frame.shape)[[1, 0, 1, 0]]
        msg = {}
       
        if pred != [None]:
            # Process detections
            for i, det in enumerate(pred):  # detections per image
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], self.frame.shape).round()
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    prediction_status=True
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    label = '%s %.2f' % (names[int(cls)], conf)
                    # get name
                    name = names[int(cls)]
                    name_list = ["Vegetable_leaf" , "Banana_peel" , "Shell" , "Plastic_bottle" , "Basketball" , "Carton" , "Bandage" , "Expired_capsule_drugs"]
                    for i in name_list:
                        if name == i:prediction_status=False
                    if prediction_status==True: 
                        point_x = np.int(xywh[0] * 640)
                        point_y = np.int(xywh[1] * 480)
                        cv.circle(self.frame, (point_x, point_y), 5, (0, 0, 255), -1)
                        plot_one_box(xyxy, self.frame, label=label, color=colors[int(cls)], line_thickness=2)
                        # Get current time
                        curr_time = time.time()
                        # Calculation time difference
                        exec_time = curr_time - prev_time
                        info = "time: %.2f ms" % (1000 * exec_time)
                       
        return msg

   