#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from std_msgs.msg import Float32,Bool
import os
from cv_bridge import CvBridge
import cv2 as cv
from Dofbot_Track import *
encoding = ['16UC1', '32FC1']
import time
#color recognition
from astra_common import *
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
import rospkg
from dofbot_pro_color.cfg import ColorHSVConfig
import math
from dofbot_pro_info.msg import *
import tf.transformations as tf
class ColorDetect:
    def __init__(self):
        nodeName = 'color_detect'
        rospy.init_node(nodeName)
        self.window_name = "depth_image"
        self.init_joints = [90.0, 150, 12.0, 20.0, 90, 30]
        self.dofbot_tracker = DofbotTrack()
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.grasp_status_sub = rospy.Subscriber('grab', Bool, self.grabStatusCallback, queue_size=1)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw',Image)
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.TimeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub,self.depth_image_sub],10,0.5)
        self.TimeSynchronizer.registerCallback(self.TrackAndGrap)
        self.cnt = 0
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()

        #color
        self.Roi_init = ()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.dyn_update = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'init'
        self.color = color_detect()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.cx = 0
        self.cy = 0
        
        self.hsv_text = rospkg.RosPack().get_path("dofbot_pro_color") + "/scripts/colorHSV.text"
        Server(ColorHSVConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client(nodeName, timeout=60)
        self.pr_time = time.time()
        self.circle_r = 0 
        self.cur_distance = 0.0
        self.corner_x = self.corner_y = 0.0
        exit_code = os.system('rosservice call /camera/set_color_exposure  50')

 

    def grabStatusCallback(self,msg):
        if msg.data == True:
            self.Track_state = 'init'


    
    def dynamic_reconfigure_callback(self, config, level):
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        write_HSV(self.hsv_text, self.hsv_range)
        return config     

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
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
        result_frame, binary = self.process(result_image,action)
        
        if self.cx!=0 and self.cy!=0 and self.circle_r>30 :
            center_x, center_y = self.cx,self.cy
            cv2.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
            self.cur_distance = depth_image_info[int(center_y),int(center_x)]/1000.0
            print("self.cur_distance: ",self.cur_distance)
            dist = round(self.cur_distance,3)
            dist = 'dist: ' + str(dist) + 'm'
            cv.putText(result_frame, dist,  (30, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

            if abs(center_x-320) >2 or abs(center_y-240)>2:
                self.dofbot_tracker.XY_track(center_x,center_y)
            else:
                self.cnt = self.cnt + 1
                if self.cnt==10:
                    self.cnt = 0
                    print("take it now!")
                    if self.cur_distance!=0:
                        angle_radians = math.atan2(self.corner_y, self.corner_x)
                        angle_degrees = math.degrees(angle_radians)
                        print("angle_degrees: ",angle_degrees)
                        if abs(angle_degrees) >90:
                            compute_angle = abs(angle_degrees) - 90
                        else:
                            compute_angle = abs(angle_degrees)
                        set_joint5 = compute_angle
                        if 50>set_joint5 and set_joint5>40:
                            print("--------------------------------------")
                            self.dofbot_tracker.set_joint5 = 90 
                        else:
                            self.dofbot_tracker.set_joint5 = set_joint5 + 40
                        self.dofbot_tracker.Clamping(center_x,center_y,self.cur_distance)

        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_frame, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)                              
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([result_frame, binary])))
        else:
            cv.imshow(self.windows_name, result_frame)
        cv2.imshow("depth_image", depth_to_color_image)


    def process(self, rgb_img, action):
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == ord('i') or action == ord('I'): self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): self.Reset()
        #elif action == ord('q') or action == ord('Q'): self.cancel()
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text): self.hsv_range = read_HSV(self.hsv_text)
            else: self.Track_state = 'init'
        if self.Track_state != 'init':
            if len(self.hsv_range) != 0:
                rgb_img, binary, self.circle ,corners= self.color.object_follow(rgb_img, self.hsv_range)
                print("corners[0]: ",corners[0][0])
                print("corners[0]: ",corners[0][1])
                self.corner_x = int(corners[0][0]) - int(self.circle[0])
                self.corner_y = int(corners[0][1]) - int(self.circle[1])

                #rgb_img, binary, self.circle = self.color.object_follow_list(rgb_img, self.hsv_range)
                #print("circle: ",self.circle)
                self.cx = self.circle[0]
                self.cy = self.circle[1]
                self.circle_r = self.circle[2]
                if self.dyn_update == True:
                    write_HSV(self.hsv_text, self.hsv_range)
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                    self.dyn_client.update_configuration(params)
                    self.dyn_update = False

        return rgb_img, binary

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        self.init_joints = [90.0, 93, 37, 0.0, 90, 90]
        #self.dofbot_tracker.pub_arm(self.init_joints)
        self.cx = 0
        self.cy = 0

        

    def calculate_yaw(self,bin_img):
        contours = cv.findContours(bin_img, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)[-2]
        c = max(contours, key = cv.contourArea)
        area = math.fabs(cv.contourArea(c))
        rect = cv.minAreaRect(c)
        

    def pub_arm(self, joints, id=6, angle=180, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        if len(joints) != 0: arm_joint.joints = joints
        else: arm_joint.joints = []
        self.pubPoint.publish(arm_joint)

if __name__ == '__main__':
    try:
        color_detect = ColorDetect()
        color_detect.pub_arm(color_detect.init_joints)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

