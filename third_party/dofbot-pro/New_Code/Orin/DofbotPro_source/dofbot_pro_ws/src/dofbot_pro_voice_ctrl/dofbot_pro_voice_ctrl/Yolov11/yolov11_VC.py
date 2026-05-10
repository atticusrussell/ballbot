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
from std_msgs.msg import Float32, Bool,Int8
from IPython.display import display
from dofbot_pro_yolov11.fps import FPS
from ultralytics import YOLO
from dofbot_pro_yolov11.robot_controller import Robot_Controller
from dofbot_pro_interface.msg import * 
encoding = ['16UC1', '32FC1']


import os
exit_code = os.system('ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 "data: 40"')

class Yolov11DetectNode(Node):
    def __init__(self):
        super().__init__('detect_node')

        self.pr_time = time.time()
        self.image_sub = self.create_subscription(ImageMsg,"/image_data",self.image_sub_callback,qos_profile=1)
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 90.0]
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", 10)
        self.pubDetect = self.create_publisher(Yolov11Detect, "Yolov11DetectInfo", 10)
        self.pub_SortFlag = self.create_publisher(Bool, 'sort_flag', 10)
        self.grasp_status_sub = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)

        self.sub_voice = self.create_subscription(Int8,"voice_result",self.getVoiceResultCallBack,1)
        self.pub_playID = self.create_publisher(Int8, "player_id", qos_profile=10)
        self.start_flag = False
        self.yolo_model = YOLO("/home/jetson/dofbot_pro_ws/src/dofbot_pro_voice_ctrl/dofbot_pro_voice_ctrl/Yolov11/best.engine", task='detect')
        self.fps = FPS()
        
        self.name = None
        self.recyclable_waste=['Newspaper','Zip_top_can','Book','Old_school_bag']
        self.toxic_waste=['Syringe','Expired_cosmetics','Used_batteries','Expired_tablets']
        self.wet_waste=['Fish_bone','Egg_shell','Apple_core','Watermelon_rind']
        self.dry_waste=['Toilet_paper','Peach_pit','Cigarette_butts','Disposable_chopsticks'] 
        self.play_id = Int8()
    
    def image_sub_callback(self,data):
        image = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8, buffer=data.data) # 将自定义图像消息转化为图像
        self.img[:,:,0],self.img[:,:,1],self.img[:,:,2] = image[:,:,2],image[:,:,1],image[:,:,0] # 将rgb 转化为opencv的bgr顺序
        # img = self.img.copy()


        results = self.yolo_model(self.img, save=False, verbose=False)  # 使用YOLO11进行物体检测
        annotated_frame = results[0].plot(
            labels = True, # 显示标签
            conf = False,  # 显示置信度
            boxes = True,  # 绘制边界框
        )
        boxes = results[0].boxes
        key = cv2.waitKey(10)
        # print("--")
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
                
                self.name =  center.result   
                cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)  # 绘制红色圆点
                cv2.putText(annotated_frame, f"({center_x}, {center_y})", (center_x + 10, center_y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1) 

                self.pubDetect.publish(center)
                self.start_flag = False
        elif boxes == [None]:
            print("Do not find the target.")
            
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(self.img, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("frame", annotated_frame)
        

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

    def getVoiceResultCallBack(self,msg):
        if msg.data == 108:
            play_id = Int8()
            play_id.data = 45
            self.pub_playID.publish(play_id)
            time.sleep(3.0)
            self.start_flag = True
            print("Start sorting garbage.")

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
