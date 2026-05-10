#!/usr/bin/env python3
# encoding: utf-8
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool,String
from cv_bridge import CvBridge
import cv2 as cv

import getpass
import threading
encoding = ['16UC1', '32FC1']
import time
import math
import os

from sensor_msgs.msg import CompressedImage,Image
from std_msgs.msg import Int32, Bool,UInt16,Int16MultiArray
from dofbot_pro_KCF.astra_common import *
from dofbot_pro_interface.msg import *

class mono_Tracker(Node):
    def __init__(self):
        super().__init__('monoIdentify') 

        self.point_pose = (0, 0, 0)
        self.circle = (0, 0, 0)
        self.hsv_range = ()
        self.circle_r = 0
        self.dyn_update = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.init_joints = [90.0, 120.0, 0, 0, 90.0, 30.0]
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.end = 0
        self.cx = 0
        self.cy = 0

        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()

        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", 10)
        self.pub_pos = self.create_publisher(Position, "/pos_xyz", 10)
        self.pub_track_pos = self.create_publisher(Position, "/pos_xy", 10)
        self.joint6_pub = self.create_publisher(Float32,'joint6',1)
        self.subscription = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)

        self.depth_image_sub  = Subscriber(self, Image, "/camera/color/image_raw", qos_profile=1)
        self.rgb_image_sub = Subscriber(self, Image, "/camera/depth/image_raw", qos_profile=1)
        self.TimeSynchronizer = ApproximateTimeSynchronizer([self.depth_image_sub, self.rgb_image_sub],queue_size=10,slop=0.1)
        self.TimeSynchronizer.registerCallback(self.kcfTrack)
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',2)

        self.xy_subscription = self.create_subscription(Int16MultiArray,'corner_xy',self.GetXYCallback,qos_profile=1)

        self.tracker_type = 'KCF'
        self.VideoSwitch = True
        self.img_flip = False

        print("OpenCV Version: ",cv.__version__)
        self.gTracker = Tracker(tracker_type=self.tracker_type)
        self.Track_state = 'init'

        self.pr_time = time.time()
        self.circle_r = 0 
        self.cur_distance = 0.0
        self.corner_x = self.corner_y = 0.0
        self.get_xy = False
        self.compute_roi = False
        self.rect = None
        self.frame_flag = True

        
    def GetXYCallback(self,msg):
        print("msg: ",msg.data)
        print(msg.data[0])
        print(msg.data[1])        
        print(msg.data[2])
        print(msg.data[3])
        self.Roi_init = (msg.data[0],msg.data[1],msg.data[2],msg.data[3])
        self.Track_state = 'identify'
        self.get_xy = True
        self.rect = [msg.data[0],msg.data[1],msg.data[2],msg.data[3]]

    
    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'


    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x,y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'identify'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
            print("self.Roi_init: ",self.Roi_init)

    def kcfTrack(self,color_frame,depth_frame):
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

        result_frame, binary = self.process(result_image,action)

        
        if self.cx!=0 and self.cy!=0 and self.circle_r>10 :
            center_x, center_y = self.cx,self.cy
            cv2.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
            self.cur_distance = depth_image_info[int(center_y),int(center_x)]/1000.0
            #print("self.cur_distance: ",self.cur_distance)
            dist = round(self.cur_distance,3)
            dist = 'dist: ' + str(dist) + 'm'
            cv.putText(result_frame, dist,  (30, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            pos = Position()
            pos.x = center_x
            pos.y = center_y
            pos.z = self.cur_distance
            Track_pos = Position()
            Track_pos.x = center_x
            Track_pos.y = center_y
            self.pub_track_pos.publish(Track_pos)
            
            
            if self.cur_distance!=0.0:
                print("self.cur_distance: ",self.cur_distance)
                if self.rect!=None:
                    if self.frame_flag == True:
                        cv2.rectangle(result_frame, (self.rect[0],self.rect[1]), (self.rect[2],self.rect[3]), (255,235,23), thickness=1, lineType=cv2.LINE_8, shift=0)
                self.pub_pos.publish(pos)
                self.frame_flag = False
            else:
                print("Invalid distance.Please move the target  to get the valid distance.")
                
          

        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_frame, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)                              
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([result_frame, binary])))
        else:
            cv.imshow(self.windows_name, result_frame)
        cv2.imshow("depth_image", depth_to_color_image)

    def process(self, rgb_img, action):
        #print("self.Track_state: ",self.Track_state)
        #print("self.Roi_init: ",self.Roi_init)
        #print("self.tracker_type: ",self.tracker_type)
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
        if action == 32: self.Track_state = 'tracking'
        elif action == ord('i') or action == 105: self.Track_state = "identify"
        elif action == ord('r') or action == 114: self.Reset()
        if self.get_xy==True:
            
            if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3] and self.compute_roi == False:
                self.gTracker_state = True
                self.dyn_update = True
    
            if self.Track_state != 'init':
                # print(self.tracker_type)
                if self.tracker_type != "color":
                    #print("*******************************")
                    if self.gTracker_state == True:
                        
                        Roi = (self.Roi_init[0], self.Roi_init[1], self.Roi_init[2] - self.Roi_init[0], self.Roi_init[3] - self.Roi_init[1])
                        self.compute_roi = True
                        print("Roi: ",Roi)
                        self.gTracker = Tracker(tracker_type=self.tracker_type)
                        self.gTracker.initWorking(rgb_img, Roi)
                        self.gTracker_state = False
                    rgb_img, (targBegin_x, targBegin_y), (targEnd_x, targEnd_y) = self.gTracker.track(rgb_img)
                    center_x = targEnd_x / 2 + targBegin_x / 2
                    center_y = targEnd_y / 2 + targBegin_y / 2
                    self.cx = center_x
                    self.cy = center_y
                    #print(self.cx)
                    #print(self.cy)
                    #print("-----------------")
                    
    
                    width = targEnd_x - targBegin_x
                    high = targEnd_y - targBegin_y
                    self.circle_r = min(width, high)
                   
                    joint_x = Float32()
                    joint_x.data = float(self.circle_r)
                    self.joint6_pub.publish(joint_x)
                    # print(self.circle_r)

        return rgb_img, binary

    def pub_arm(self, joints, id=6, angle=180.0, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        arm_joint.joints = joints
        self.pubPoint.publish(arm_joint)

    def GraspStatusCallback(self,msg):
         
        if msg.data == True:
            print("grasp is done.")
            self.largemodel_arm_done_pub.publish(String(data="grasp_obj_done"))
            self.Reset()

def main(args=None):
    rclpy.init(args=args)
    kcf_tracker = mono_Tracker()
    try:
        rclpy.spin(kcf_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        kcf_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


