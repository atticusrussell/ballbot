#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int8, Bool
from dt_apriltags import Detector
from dofbot_pro_apriltag.vutils import draw_tags
from cv_bridge import CvBridge
import cv2 as cv
from dofbot_pro_interface.srv import *  # 假设服务已迁移
from dofbot_pro_interface.msg import ArmJoint, AprilTagInfo
import pyzbar.pyzbar as pyzbar
import time
import queue
import os
encoding = ['16UC1', '32FC1']
class AprilTagDetectNode(Node):  # 继承自Node
    def __init__(self):
        super().__init__('apriltag_detect_node')  # ROS2节点初始化
        
        # 初始化参数
        self.detect_joints = [90.0, 150.0, 12.0, 20.0, 90.0, 30.0]
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 90.0]
        self.search_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
    
        
        # ROS2 发布者
        self.pubGraspStatus = self.create_publisher(Bool, "grasp_done", 1)
        self.tag_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", 1)
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", 1)
        
        # ROS2 订阅者（消息同步）
        self.depth_image_sub  = Subscriber(self, Image, "/camera/color/image_raw", qos_profile=1)
        self.rgb_image_sub = Subscriber(self, Image, "/camera/depth/image_raw", qos_profile=1)
        self.TimeSynchronizer = ApproximateTimeSynchronizer([self.depth_image_sub, self.rgb_image_sub],queue_size=10,slop=0.5)
        self.TimeSynchronizer.registerCallback(self.TagDetect)

        # ROS2 其他订阅者
        self.grasp_status_sub = self.create_subscription(Bool, 'grasp_done', self.GraspStatusCallback, 1)
        self.sub_targetID = self.create_subscription(Int8, "TargetId", self.GetTargetIDCallback, 1)
        
        # 初始化工具
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.pubPos_flag = False
        
        # AprilTag检测器（配置保持不变）
        self.at_detector = Detector(
            searchpath=['apriltags'],
            families='tag36h11',
            nthreads=8,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        
        # 状态变量
        self.pr_time = time.time()
        self.target_id = 0
        self.cnt = 0
        self.detect_flag = False
        self.pub_arm(self.detect_joints)

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
        if key == 32:
            self.pubPos_flag = True
        # print("1")
        if self.target_id!=0:
            if len(tags) > 0 :
                # print("1")
                for i in range(len(tags)):
                    center_x, center_y = tags[i].center
                    # print(self.pubPos_flag)
                    if self.pubPos_flag == True:
                        center_x, center_y = tags[i].center
                        cv.circle(result_image, (int(center_x),int(center_y)), 10, (0,210,255), thickness=-1)
                        cv.putText(result_image, str(self.target_id), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        
                        print("center_x, center_y: ",center_x, center_y)
                        print("depth: ",depth_image_info[int(center_y),int(center_x)]/1000)
                        tag = AprilTagInfo()
                        tag.id = tags[i].tag_id
                        tag.x = center_x
                        tag.y = center_y
                        tag.z = depth_image_info[int(center_y),int(center_x)]/1000
                        print("tag_id: ",tags[i].tag_id)
                        print("target_id: ",self.target_id)
                        if tag.z>0 and tag.id == self.target_id:
                            self.tag_info_pub.publish(tag)
                            self.pubPos_flag = False
                            self.cnt = 0
                            self.detect_flag = True
                            print("********************************************")
                        else:
                            if tag.z!=0:
                                print("Invalid distance.")
                            if tag.id != self.target_id:
                                print("Do not find the target id tag.")
                                grasp_done = Bool()
                                grasp_done.data = True
                                self.pubGraspStatus.publish(grasp_done)
                                self.cnt = self.cnt + 1
                                self.detect_flag = False
                                self.pubPos_flag = True
                                if self.cnt == 20:
                                    self.cnt = 0

                if self.detect_flag != True  and self.pubPos_flag==True:
                    self.pub_arm(self.detect_joints)
                    self.target_id = 0
                    grasp_done = Bool()
                    grasp_done.data = True
                    self.pubGraspStatus.publish(grasp_done)
                    self.detect_flag  = False
                                    
            elif self.pubPos_flag == True and len(tags) == 0:
                grasp_done = Bool()
                grasp_done.data = True
                self.pubGraspStatus.publish(grasp_done)
                self.pub_arm(self.detect_joints)
                
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("result_image", result_image)
        key = cv2.waitKey(1)

    def pub_arm(self, joints, id=6, angle=180.0, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        arm_joint.joints = joints
        self.pubPoint.publish(arm_joint)

    def GraspStatusCallback(self,msg):
        if msg.data == True:
            self.pubPos_flag = False

    def GetTargetIDCallback(self,msg):
        self.target_id = msg.data
        print("Get th traget is ",self.target_id)
        self.pub_arm(self.init_joints)

def main(args=None):
    rclpy.init(args=args)
    tag_detect = AprilTagDetectNode()
    try:
        rclpy.spin(tag_detect)
    except KeyboardInterrupt:
        pass
    finally:
        tag_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()