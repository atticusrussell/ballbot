#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import os
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
from dofbot_pro_info.msg import ArmJoint
from dofbot_pro_info.msg import AprilTagInfo
from std_msgs.msg import Float32,Bool
encoding = ['16UC1', '32FC1']
import time
import queue


class AprilTagDetectNode:
    def __init__(self):
        rospy.init_node('apriltag_detect')
        self.init_joints = [90.0, 120, 0.0, 0.0, 90, 30]
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw',Image)
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.tag_info_pub = rospy.Publisher("TagInfo",AprilTagInfo,queue_size=1)
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.TimeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub,self.depth_image_sub],1,0.5)
        self.grasp_status_sub = rospy.Subscriber('grasp_done', Bool, self.GraspStatusCallback, queue_size=1)
        self.TimeSynchronizer.registerCallback(self.TagDetect)
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.pubPos_flag = True
        self.pr_time = time.time()
        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        exit_code = os.system('rosservice call /camera/set_color_exposure  50')

    def TagDetect(self,color_frame,depth_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        result_image = np.copy(rgb_image)
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
        tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
        draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
        key = cv2.waitKey(10)

        if len(tags) > 0 :
            for i in range(len(tags)):
                if self.pubPos_flag == True:
                    center_x, center_y = tags[i].center
                    cv2.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
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
                    else:
                        print("Invalid distance.")
                        
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)                   
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("result_image", result_image)
        cv2.imshow("depth_image", depth_to_color_image)
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

