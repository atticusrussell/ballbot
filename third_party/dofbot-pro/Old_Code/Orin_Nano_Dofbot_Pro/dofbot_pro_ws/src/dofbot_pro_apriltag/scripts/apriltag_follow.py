#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2 
import rospy
import numpy as np
from sensor_msgs.msg import Image
import os
import time
from vutils import draw_tags
from dt_apriltags import Detector
from Dofbot_Track import *
import message_filters
from cv_bridge import CvBridge
from std_msgs.msg import Float32
encoding = ['16UC1', '32FC1']
class TagTrackingNode:
    def __init__(self):
        rospy.init_node('apriltag_tracking')
        self.dofbot_tracker = DofbotTrack()
        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        self.cnt = 0
        self.cur_distance = 0.0
        self.pr_time = time.time()
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        #self.sub_depth = rospy.Subscriber("depth",Float32,self.distCallBack)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw',Image)
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.TimeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub,self.depth_image_sub],1,0.5)
        self.TimeSynchronizer.registerCallback(self.DetectAndRemove)
        exit_code = os.system('rosservice call /camera/set_color_exposure  50')

    def DetectAndRemove(self,color_frame,depth_frame):
        # 将画面转为 opencv 格式
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        result_image = np.copy(rgb_image)
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1.0), cv2.COLORMAP_JET)
        frame = cv2.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        
        tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
        tags = sorted(tags, key=lambda tag: tag.tag_id) 
        draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))

        if len(tags) > 0 :
            #print("tag: ",tags)
            center_x, center_y = tags[0].center
            self.cur_distance = depth_image_info[int(center_y),int(center_x)]/1000.0
            dist = round(self.cur_distance,3)
            dist = 'dist: ' + str(dist) + 'm'
            cv2.putText(result_image, dist,  (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv2.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
            #print("cur_distance: ",self.cur_distance)
            #XY move
            if abs(center_x-320) >10 or abs(center_y-240)>10:
                self.cnt = 0
                self.dofbot_tracker.XY_track(center_x,center_y)
            #self.cur_x = center_x
            #self.cur_y = center_y
            #print("=====================================")
            if abs(center_x-320) <10 and abs(center_y-240)<10:
                print("center_x: ",center_x)
                print("center_y: ",center_y)
                self.cnt = self.cnt + 1 
                if self.cnt==10:
                    self.cnt = 0
                    print("take it now!")
                    print("last_joint: ",self.dofbot_tracker.cur_joints)
                    if self.cur_distance!=0:
                        vx = int(tags[0].corners[0][0]) - int(center_x)
                        vy = int(tags[0].corners[0][1]) - int(center_y)
                        angle_radians = math.atan2(vy, vx)
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
				
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("result_image", result_image)
        cv2.imshow("depth_image", depth_to_color_image)
        key = cv2.waitKey(1)
    



if __name__ == '__main__':
    try:
        tag_tracking = TagTrackingNode()
        tag_tracking.dofbot_tracker.pub_arm(tag_tracking.dofbot_tracker.init_joints)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

