#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from std_msgs.msg import Float32
from vutils import draw_tags
from dt_apriltags import Detector
from cv_bridge import CvBridge
import cv2 as cv
from dofbot_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse
from dofbot_pro_info.msg import *
import pyzbar.pyzbar as pyzbar
from std_msgs.msg import Float32,Bool
encoding = ['16UC1', '32FC1']
import time
import queue
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

def angle_between_points(x1, y1, x2, y2, x3, y3):
    # Vectors AB and BC
    ABx = x2 - x1
    ABy = y2 - y1
    BCx = x3 - x2
    BCy = y3 - y2
    
    # Calculate dot product of AB and BC
    dot_product = ABx * BCx + ABy * BCy
    
    # Calculate magnitudes of AB and BC
    magnitude_AB = math.sqrt(ABx**2 + ABy**2)
    magnitude_BC = math.sqrt(BCx**2 + BCy**2)
    
    # Calculate cosine of the angle between  AB and BC
    cos_theta = dot_product / (magnitude_AB * magnitude_BC)
    
    # Calculate the angle in radians
    theta_rad = math.acos(cos_theta)
    
    # Convert angle to degrees
    theta_deg = math.degrees(theta_rad)
    
    return theta_deg


class AprilTagDetectNode:
    def __init__(self):
        rospy.init_node('apriltag_detect')
        #self.init_joints = [90.0, 120, 0, 0.0, 90, 90]
        self.init_joints = [90.0, 150, 10, 20.0, 90, 30]
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw',Image)
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.tag_info_pub = rospy.Publisher("TagInfo",AprilTagInfo,queue_size=1)
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.TimeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub,self.depth_image_sub],1,0.5)
        self.grasp_status_sub = rospy.Subscriber('grasp_done', Bool, self.GraspStatusCallback, queue_size=1)
        self.TimeSynchronizer.registerCallback(self.TagDetect)
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.pubPos_flag = False
        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        self.target_id = 31
        self.Center_x_list = []
        self.Center_y_list = []

    def TagDetect(self,color_frame,depth_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        result_image = np.copy(rgb_image)
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
        tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
        draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
        key = cv2.waitKey(10)
        '''self.Center_x_list = list(range(len(tags)))
        self.Center_y_list = list(range(len(tags)))
        print(self.Center_x_list)
        print(self.Center_y_list)'''
        if key == 32:
            self.pubPos_flag = True
        if len(tags) > 0 :
            for i in range(len(tags)):
                center_x, center_y = tags[i].center
                #print("tag_id: ",tags[i].tag_id)
                print("center_x, center_y: ",center_x, center_y)
                print("depth: ",depth_image_info[int(center_y),int(center_x)]/1000)

                #self.tag_info_pub.publish(tag)
                if self.pubPos_flag == True:
                    center_x, center_y = tags[i].center
                    cv.circle(result_image, (int(center_x),int(center_y)), 10, (0,210,255), thickness=-1)
                    print("tag_id: ",tags[i].tag_id)
                    print("center_x, center_y: ",center_x, center_y)
                    print("depth: ",depth_image_info[int(center_y),int(center_x)]/1000)
                    tag = AprilTagInfo()
                    tag.id = tags[i].tag_id
                    tag.x = center_x
                    tag.y = center_y
                    tag.z = depth_image_info[int(center_y),int(center_x)]/1000
                    if tag.z!=0:
                        self.tag_info_pub.publish(tag)
                        self.pubPos_flag = True
                    else:
                        print("Invalid distance.")
                           
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("result_image", result_image)
        key = cv2.waitKey(1)

    
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
            self.pubPos_flag = True
if __name__ == '__main__':
    try:
        tag_detect = AprilTagDetectNode()
        tag_detect.pub_arm(tag_detect.init_joints)
        #tag_tracking.dofbot_tracker.compute_set_joints()
        '''while not rospy.is_shutdown():
            tag_tracking.node_proc()'''
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

