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
encoding = ['16UC1', '32FC1']
import time
#color recognition
from astra_common import *
#from shape_recognize import *
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
import rospkg
from dofbot_pro_color.cfg import ColorHSVConfig
import math
from dofbot_pro_info.msg import *

class TagTrackingNode:
    def __init__(self):
        nodeName = 'shape_detect'
        rospy.init_node(nodeName)

        self.target_servox=90
        self.window_name = "depth_image"
        self.target_servoy=45


        self.init_joints = [90.0, 120, 0.0, 0.0, 90, 90]

        self.pubDist = rospy.Publisher("Distance", Float32, queue_size=1)
        self.pub_ColorInfo = rospy.Publisher("PosInfo", AprilTagInfo, queue_size=1)
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)

        self.grasp_status_sub = rospy.Subscriber('grasp_done', Bool, self.GraspStatusCallback, queue_size=1)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw',Image)
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.TimeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub,self.depth_image_sub],10,0.5)
        self.TimeSynchronizer.registerCallback(self.TrackAndGrap)
        
        self.y = 320 #320
        self.x = 240 #240
        self.pr_time = time.time()
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
        self.hsv_text = rospkg.RosPack().get_path("dofbot_pro_color") + "/scripts/colorHSV.text"
        Server(ColorHSVConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client(nodeName, timeout=60)
        self.cx = 0
        self.cy = 0
        self.circle_r = 0 #防止误识别到其他的杂乱的点
        self.pubPos_flag = False
        exit_code = os.system('rosservice call /camera/set_color_exposure  50')
        self.color.target_shape = rospy.get_param("~Shape", "Square") # "Rectangle" ,"Cylinder"
        print("Target shape: ",self.color.target_shape)
        
    def GraspStatusCallback(self,msg):
        if msg.data == True:
            self.pubPos_flag = True

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
        if self.color.shape_cx!=0 and self.color.shape_cy!=0:
            cv2.circle(depth_to_color_image,(int(self.color.shape_cx),int(self.color.shape_cy)),1,(255,255,255),10)
            pos = AprilTagInfo()
            pos.x = self.color.shape_cx
            pos.y = self.color.shape_cy
            print(self.color.shape_cx,self.color.shape_cy)
            pos.z = depth_image_info[self.color.shape_cy,self.color.shape_cx]/1000
            print("depth: ",pos.z)
            if self.pubPos_flag == True and pos.z!=0:
                self.pub_ColorInfo.publish(pos)
                self.pubPos_flag = False
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
        if action == 32: self.pubPos_flag = True
        elif action == ord('i') or action == ord('I'): self.Track_state = "identify"
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
                rgb_img, binary, self.circle = self.color.ShapeRecognition(rgb_img, self.hsv_range)
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
        self.pubPos_flag = False
        

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
        tag_tracking = TagTrackingNode()
        tag_tracking.pub_arm(tag_tracking.init_joints)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

