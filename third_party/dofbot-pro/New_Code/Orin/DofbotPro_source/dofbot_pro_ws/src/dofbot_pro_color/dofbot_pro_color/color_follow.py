#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2 as cv

encoding = ['16UC1', '32FC1']
import time
import math
import os
#color recognition
from dofbot_pro_color.astra_common import *
from dofbot_pro_interface.msg import *
from dofbot_pro_color.Dofbot_Track import *
import tf_transformations as tf
import transforms3d as tfs

class ColorDetect(Node):
    def __init__(self):
        super().__init__('color_detect') 
        self.declare_param()
        self.window_name = "depth_image"
        self.init_joints = [90.0, 150.0, 12.0, 20.0, 90.0, 30.0]
        self.dofbot_tracker = DofbotTrack()
        self.cx = 0
        self.cy = 0
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", 1)
        self.grasp_status_sub = self.create_subscription(Bool, 'grab', self.grabStatusCallback, 1)

        self.depth_image_sub  = Subscriber(self, Image, "/camera/color/image_raw", qos_profile=1)
        self.rgb_image_sub = Subscriber(self, Image, "/camera/depth/image_raw", qos_profile=1)
        self.TimeSynchronizer = ApproximateTimeSynchronizer([self.depth_image_sub, self.rgb_image_sub],queue_size=10,slop=0.5)
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
        
        
        self.hsv_text = "/home/jetson/dofbot_pro_ws/src/dofbot_pro_color/dofbot_pro_color/colorHSV.text"
        if os.path.exists(self.hsv_text): self.roi_hsv_range = read_HSV(self.hsv_text)
        self.pr_time = time.time()
        self.circle_r = 0 
        self.cur_distance = 0.0
        self.corner_x = self.corner_y = 0.0


 

    def grabStatusCallback(self,msg):
        if msg.data == True:
            self.Track_state = 'init'

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

    def declare_param(self):
        #HSV
        self.declare_parameter("Hmin",0)
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.declare_parameter("Smin",85)
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.declare_parameter("Vmin",126)
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.declare_parameter("Hmax",9)
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.declare_parameter("Smax",253)
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.declare_parameter("Vmax",253)
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
        self.declare_parameter('refresh',False)
        self.refresh = self.get_parameter('refresh').get_parameter_value().bool_value

    
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
            # print("self.cur_distance: ",self.cur_distance)
            dist = round(self.cur_distance,3)
            dist = 'dist: ' + str(dist) + 'm'
            cv.putText(result_frame, dist,  (30, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            # print("x:",center_x-320)
            # print("x:",center_y-240)
            if abs(center_x-320) >3 or abs(center_y-240)>3:
                self.dofbot_tracker.XY_track(center_x,center_y)
            else:
                print(self.cnt)
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
        self.get_param()
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == ord('i') or action == ord('I'): self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): self.Reset()

        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.roi_hsv_range = self.hsv_range
                    self.dyn_update = True
                else: self.Track_state = 'init'

        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text): self.hsv_range = read_HSV(self.hsv_text)
            else: self.Track_state = 'init'

        if self.Track_state != 'init':
            if len(self.hsv_range) != 0:
                rgb_img, binary, self.circle ,corners= self.color.object_follow(rgb_img, self.hsv_range)
                # print("corners[0]: ",corners[0][0])
                # print("corners[0]: ",corners[0][1])
                self.corner_x = int(corners[0][0]) - int(self.circle[0])
                self.corner_y = int(corners[0][1]) - int(self.circle[1])
                self.cx = self.circle[0]
                self.cy = self.circle[1]
                self.circle_r = self.circle[2]
                if self.dyn_update == True:
                    write_HSV(self.hsv_text, self.hsv_range)
                    self.Hmin  = rclpy.parameter.Parameter('Hmin',rclpy.Parameter.Type.INTEGER,self.roi_hsv_range[0][0])
                    self.Smin  = rclpy.parameter.Parameter('Smin',rclpy.Parameter.Type.INTEGER,self.roi_hsv_range[0][1])
                    self.Vmin  = rclpy.parameter.Parameter('Vmin',rclpy.Parameter.Type.INTEGER,self.roi_hsv_range[0][2])
                    self.Hmax  = rclpy.parameter.Parameter('Hmax',rclpy.Parameter.Type.INTEGER,self.roi_hsv_range[1][0])
                    self.Smax  = rclpy.parameter.Parameter('Smax',rclpy.Parameter.Type.INTEGER,self.roi_hsv_range[1][1])
                    self.Vmax  = rclpy.parameter.Parameter('Vmax',rclpy.Parameter.Type.INTEGER,self.roi_hsv_range[1][2])
                    all_new_parameters = [self.Hmin,self.Smin,self.Vmin,self.Hmax,self.Smax,self.Vmax]
                    self.set_parameters(all_new_parameters)
                    self.dyn_update = False

        return rgb_img, binary

    def get_param(self):
        #hsv
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
        self.refresh = self.get_parameter('refresh').get_parameter_value().bool_value
        self.hsv_range = ((int(self.Hmin), int(self.Smin), int(self.Vmin)), (int(self.Hmax), int(self.Smax), int(self.Vmax)))
    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        self.init_joints = [90.0, 93.0, 37.0, 0.0, 90.0, 90.0]
        self.cx = 0
        self.cy = 0

        

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


