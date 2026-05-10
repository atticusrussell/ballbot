#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import torch
import numpy as np
from numpy import random
from utils.plots import plot_one_box
from models.experimental import attempt_load
from utils.general import (non_max_suppression, scale_coords, xyxy2xywh)
from utils.torch_utils import select_device, time_synchronized
import time
import message_filters
#ros
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
#from yahboomcar_msgs.msg import Image_Msg
#from yahboomcar_msgs.msg import Yolov5Detect
from dofbot_pro_info.msg import *
# Initialize
device = select_device()
# Load model
model_path = 'model0.pt'
model = attempt_load(model_path, map_location=device)  # load FP32 model
# Get names and colors
names = model.module.names if hasattr(model, 'module') else model.names
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]


encoding = ['16UC1', '32FC1']

class Yolov5DetectNode:
    def __init__(self):
        rospy.init_node('detect_node')
        self.pr_time = time.time()
        #self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw',Image)
        self.image_sub = rospy.Subscriber("/image_data",Image_Msg,self.image_sub_callback)
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像
        self.init_joints = [90.0, 120, 0.0, 0.0, 90, 90]
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.pubDetect = rospy.Publisher("Yolov5DetectInfo", Yolov5Detect, queue_size=1)
        self.pub_SortFlag = rospy.Publisher('sort_flag',Bool,queue_size=1)
        self.grasp_status_sub = rospy.Subscriber('grasp_done', Bool, self.GraspStatusCallback, queue_size=1)
        self.start_flag = False
    
    def image_sub_callback(self,data):
        image = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8, buffer=data.data) # 将自定义图像消息转化为图像
        self.img[:,:,0],self.img[:,:,1],self.img[:,:,2] = image[:,:,2],image[:,:,1],image[:,:,0] # 将rgb 转化为opencv的bgr顺序
        img = self.img.copy()
        img = np.transpose(img, (2, 0, 1))
        img = torch.from_numpy(img).to(device)
        img = img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        pred = model(img)[0]
        pred = non_max_suppression(pred, 0.4, 0.5)
        gn = torch.tensor(self.img.shape)[[1, 0, 1, 0]]
        key = cv2.waitKey(10)
        if pred != [None]:
            for i, det in enumerate(pred):  # detections per image
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], self.img.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    label = '%s %.2f' % (names[int(cls)], conf)
                    plot_one_box(xyxy, self.img, label=label, color=colors[int(cls)], line_thickness=3)
                    cx = (xyxy[0].item() + xyxy[2].item())/2
                    cy = (xyxy[1].item() + xyxy[3].item())/2
                    center = Yolov5Detect()
                    center.centerx = cx
                    center.centery = cy
                    center.result = names[int(cls)]
                    if conf.item()>0.8 :
                        self.pubDetect.publish(center)
                        
                    '''print("x1",xyxy[0].item())
                    print("y1",xyxy[1].item())
                    print("x2",xyxy[2].item())
                    print("y2",xyxy[3].item())
                    print("label: ",names[int(cls)])
                    print("score: ",conf.item())
                    print("center: ",cx,cy)'''
                    #cv2.circle(self.img, (cx,cy), 5, (0,255,255), thickness=-1)
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(self.img, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("frame", self.img)
        
        if key == 32 or self.start_flag == True:
            print("Send a start signal.")
            start_flag = Bool()
            start_flag.data = True
            self.pub_SortFlag.publish(start_flag)

    def pub_arm(self, joints, id=6, angle=180, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        if len(joints) != 0: arm_joint.joints = joints
        else: arm_joint.joints = []
        self.pubPoint.publish(arm_joint)        
        
    def GraspStatusCallback(self,msg):
        if msg.data == True:
            self.start_flag = True        
        
if __name__ == '__main__':
    try:
        yolov5_detect = Yolov5DetectNode()
        yolov5_detect.pub_arm(yolov5_detect.init_joints)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
    