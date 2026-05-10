#!/usr/bin/env python
# -*- coding: utf-8 -*-
print("语音指令词如下：")
print("------分拣红色块、分拣绿色块、分拣蓝色块、分拣黄色块------")
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool,Int8
from cv_bridge import CvBridge
import cv2 as cv

import time
import math
import os
encoding = ['16UC1', '32FC1']
import tf_transformations as tf
import transforms3d as tfs

from dofbot_pro_voice_ctrl.Color.astra_common import *
from dofbot_pro_interface.msg import *

from Arm_Lib import Arm_Device
import os
exit_code = os.system('ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 "data: 40"')

class ColorDetect(Node):
    def __init__(self):
        super().__init__('color_detect')
        self.Arm = Arm_Device()
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 90.0]
        self.depth_image_sub  = Subscriber(self, Image, "/camera/color/image_raw", qos_profile=1)
        self.rgb_image_sub = Subscriber(self, Image, "/camera/depth/image_raw", qos_profile=1)
        self.TimeSynchronizer = ApproximateTimeSynchronizer([self.depth_image_sub, self.rgb_image_sub],queue_size=10,slop=0.5)
        self.TimeSynchronizer.registerCallback(self.TrackAndGrap)

        self.sub_voice = self.create_subscription(Int8,"voice_result",self.getVoiceResultCallback,1)
        
        self.grasp_status_sub = self.create_subscription(Bool, 'grasp_done', self.GraspStatusCallback, 1)
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", 1)

        self.pub_ColorInfo = self.create_publisher(Position,"xyz", 1)
        self.pub_playID = self.create_publisher(Int8,"player_id", 1)

        self.y = 320 #320
        self.x = 240 #240
        self.pr_time = time.time()

        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()

        #color
        self.Roi_init = ()
        self.hsv_range = ()
        self.roi_hsv_range = ()
        self.circle = (0, 0, 0)
        self.dyn_update = False
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'init'
        self.color_list = ['red','green','blue','yellow']
        self.index = 0
        self.color = color_detect()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.cx = 0
        self.cy = 0
        self.target_color = "red"
        self.play_id = Int8()

        self.red_hsv_text = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_voice_ctrl/dofbot_pro_voice_ctrl/Color/red_colorHSV.text"
        self.green_hsv_text = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_voice_ctrl/dofbot_pro_voice_ctrl/Color/green_colorHSV.text"
        self.blue_hsv_text = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_voice_ctrl/dofbot_pro_voice_ctrl/Color/blue_colorHSV.text"
        self.yellow_hsv_text = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_voice_ctrl/dofbot_pro_voice_ctrl/Color/yellow_colorHSV.text"
        #if os.path.exists(self.red_hsv_text): self.roi_hsv_range = read_HSV(self.red_hsv_text)
        self.circle_r = 0 #防止误识别到其他的杂乱的点
        self.pubPos_flag = False
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
        self.start_flag = False
 
    def GraspStatusCallback(self,msg):
        if msg.data == True:
            self.pubPos_flag = True

    def getVoiceResultCallback(self,msg): 
        if msg.data == 100:
            self.target_color = "red"
            self.pubPos_flag = True
        elif msg.data == 101:
            self.target_color = "green"
            self.pubPos_flag = True
        elif msg.data == 102:
            self.target_color = "blue"
            self.pubPos_flag = True
        elif msg.data == 103:
            self.target_color = "yellow"
            self.pubPos_flag = True
        play_id = Int8()
        play_id.data = 45
        self.pub_playID.publish(play_id)
        print("Get the target color is ",self.target_color)
        self.Track_state = "identify"
        self.start_flag = True
        
    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    
    def TrackAndGrap(self,color_frame,depth_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'bgr8')
        result_image = np.copy(rgb_image)
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        action = cv.waitKey(10) & 0xFF
        result_image = cv.resize(result_image, (640, 480))
        print("self.hsv_range: ",self.hsv_range)
        print("self.target_color: ",self.target_color)
        
        result_frame, binary = self.process(result_image,action)
        
        if self.cx!=0 and self.cy!=0 and self.circle_r>30 and self.start_flag == True:
            cv2.circle(depth_to_color_image,(int(self.cx),int(self.cy)),1,(255,255,255),10)
            if self.cx<=640 or self.cy <=480:
                self.start_flag = False
                center_x, center_y = self.cx,self.cy
                self.x = int(center_x)
                self.y = int(center_y)
                pos = Position()
                pos.x = float(center_x)
                pos.y = float(center_y)
                pos.z = depth_image_info[self.y,self.x]/1000
                if self.pubPos_flag == True:
                    self.pub_ColorInfo.publish(pos)
                    self.pubPos_flag = False
                    
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_frame, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)        
        cv.putText(result_frame, self.target_color, (50,50), cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1, cv.LINE_AA)
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([result_frame, binary])))
        else:
            cv.imshow(self.windows_name, result_frame)
        cv2.imshow("depth_image", depth_to_color_image)


    def process(self, rgb_img, action):
        #self.get_param()
        print("self.Track_state: ",self.Track_state)
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []

        if action == ord('i') or action == ord('I'): self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): self.Reset()
        elif action == ord('f') or action == ord('F'):
            self.index = self.index + 1
            if self.index == 4:
                self.index = 0
            self.target_color = self.color_list[self.index]

        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    if self.target_color == 'red' :
                        write_HSV(self.red_hsv_text, self.hsv_range)
                        print("red")
                    elif self.target_color == 'green':
                        write_HSV(self.green_hsv_text, self.hsv_range)
                        print("green")
                    elif self.target_color == 'blue':
                        write_HSV(self.blue_hsv_text, self.hsv_range)
                        print("blue")          
                    elif self.target_color == 'yellow':
                        write_HSV(self.yellow_hsv_text, self.hsv_range)
                        print("yellow")
                    self.roi_hsv_range = self.hsv_range

                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'

        elif self.Track_state == "identify":
            if os.path.exists(self.red_hsv_text) and self.target_color == 'red':
                self.hsv_range = read_HSV(self.red_hsv_text)
            elif os.path.exists(self.green_hsv_text) and self.target_color == 'green':
                self.hsv_range = read_HSV(self.green_hsv_text)
                
            elif os.path.exists(self.blue_hsv_text) and self.target_color == 'blue':
                self.hsv_range = read_HSV(self.blue_hsv_text)
            elif os.path.exists(self.yellow_hsv_text) and self.target_color == 'yellow':
                self.hsv_range = read_HSV(self.yellow_hsv_text)
            else: self.Track_state = 'init'

        if self.Track_state != 'init':
            if len(self.hsv_range) != 0:
                rgb_img, binary, self.circle, _= self.color.object_follow(rgb_img, self.hsv_range)

                self.cx = self.circle[0]
                self.cy = self.circle[1]
                self.circle_r = self.circle[2]
        
        return rgb_img, binary


    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        self.init_joints = [90.0, 93.0, 37.0, 0.0, 90.0, 90.0]
        self.cx = 0
        self.cy = 0
        self.pubPos_flag = False
        

    def calculate_yaw(self,bin_img):
        contours = cv.findContours(bin_img, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)[-2]
        c = max(contours, key = cv.contourArea)
        area = math.fabs(cv.contourArea(c))
        rect = cv.minAreaRect(c)
        

    def pub_arm(self, joints, id=6, angle=180.0, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        arm_joint.joints = joints
        self.pubPoint.publish(arm_joint)


def main(args=None):
    rclpy.init(args=args)
    color_detect = ColorDetect()
    color_detect.pub_arm(color_detect.init_joints)
    try:
        rclpy.spin(color_detect)
    except KeyboardInterrupt:
        pass
    finally:
        color_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
