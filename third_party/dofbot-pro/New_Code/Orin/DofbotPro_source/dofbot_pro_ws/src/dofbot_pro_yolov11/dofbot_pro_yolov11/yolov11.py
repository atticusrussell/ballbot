#!/usr/bin/env python
# coding: utf-8
import rclpy
from rclpy.node import Node
import Arm_Lib
import os
import time
import cv2 
import cv2 as cv
import numpy as np
import threading
from time import sleep
import ipywidgets as widgets
from std_msgs.msg import Float32, Bool ,String
from IPython.display import display
from dofbot_pro_yolov11.fps import FPS
from ultralytics import YOLO
from dofbot_pro_yolov11.robot_controller import Robot_Controller
from dofbot_pro_interface.msg import * 
encoding = ['16UC1', '32FC1']



class Yolov11DetectNode(Node):
    def __init__(self):
        super().__init__('detect_node')

        self.pr_time = 0
        self.image_sub = self.create_subscription(ImageMsg,"/image_data",self.image_sub_callback,qos_profile=1)
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 90.0]
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", 10)
        self.pubDetect = self.create_publisher(Yolov11Detect, "Yolov11DetectInfo", 10)
        self.pub_SortFlag = self.create_publisher(Bool, 'sort_flag', 10)
        self.grasp_status_sub = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',1)
        self.start_flag = False
        self.yolo_model = YOLO("/home/jetson/dofbot_pro_ws/src/dofbot_pro_yolov11/dofbot_pro_yolov11/best.engine", task='detect')
        self.fps = FPS()
    
    def image_sub_callback(self,data):
        image = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8, buffer=data.data) # 将自定义图像消息转化为图像
        self.img[:,:,0],self.img[:,:,1],self.img[:,:,2] = image[:,:,2],image[:,:,1],image[:,:,0] # 将rgb 转化为opencv的bgr顺序
        # self.img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)


        results = self.yolo_model(self.img, save=False, verbose=False)  # 使用YOLO11进行物体检测
        annotated_frame = results[0].plot(
            labels = True, # 显示标签
            conf = False,  # 显示置信度
            boxes = True,  # 绘制边界框
        )
        boxes = results[0].boxes
        key = cv2.waitKey(10)
        #print("boxes: ",results[0].boxes.cls.cpu().tolist())
        if boxes != [None] and self.start_flag == True:
            for box in boxes:  # detections per image
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                class_id = int(box.cls)
                confidence = float(box.conf)
                label = f"{self.yolo_model.names[class_id]} {confidence:.2f}"
                # 计算重心位置
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2

                center = Yolov11Detect()
                center.centerx = float(center_x)
                center.centery = float(center_y)
                center.result = str(self.yolo_model.names[class_id])

                # cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)  # 绘制红色圆点
                cv2.putText(annotated_frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  # 在左上角显

                self.pubDetect.publish(center)
                self.start_flag = False
        if results[0].boxes.cls.cpu().tolist() == [] and self.start_flag==True:
            print("Do not find any target.")
            self.largemodel_arm_done_pub.publish(String(data='garbage_sort_done'))
            self.start_flag = False
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(annotated_frame, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("frame", annotated_frame)
        
        if key == 32:
            print("Send a start signal.")
            start_flag = Bool()
            start_flag.data = True
            self.pub_SortFlag.publish(start_flag)
            self.start_flag = True

    def pub_arm(self, joints, id=6, angle=180.0, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        arm_joint.joints = joints
        self.pubPoint.publish(arm_joint)        
        
    def GraspStatusCallback(self,msg):
        if msg.data == True:
            self.start_flag = True
            print("Next Publish.")	

def main(args=None):
    rclpy.init(args=args)
    yolov11_detect = Yolov11DetectNode()
    yolov11_detect.pub_arm(yolov11_detect.init_joints)
    try:
        rclpy.spin(yolov11_detect)
    except KeyboardInterrupt:
        pass
    finally:
        yolov11_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()