#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2 
import numpy as np
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float32, Bool,Int8
from cv_bridge import CvBridge
import cv2 as cv
from dt_apriltags import Detector
import threading
from dofbot_pro_driver.vutils import draw_tags
from dofbot_pro_interface.srv import Kinemarics
from dofbot_pro_interface.msg import *
import pyzbar.pyzbar as pyzbar
from std_msgs.msg import Float32,Bool
import time
import queue
from Arm_Lib import Arm_Device
import os
exit_code = os.system('ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 "data: 40"')

encoding = ['16UC1', '32FC1']
class AprilTagDetectNode(Node):
    def __init__(self):
        super().__init__('apriltag_detect')
        self.Arm = Arm_Device()
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
        self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub],queue_size=10,slop=0.5)
        self.ts.registerCallback(self.TagDetect)

        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", qos_profile=1)
        self.subscription = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)
        self.sub_voice = self.create_subscription(Int8,"voice_result",self.getVoiceResultCallBack,1)
        self.pub_playID = self.create_publisher(Int8,"player_id", 1)
        
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.pubPos_flag = False
        self.pr_time = time.time()
        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)

    def getVoiceResultCallBack(self,msg):
        if msg.data == 95 or msg.data == 96 or msg.data == 97 or msg.data == 98:
            self.pubPos_flag = True
            play_id = Int8()
            play_id.data = 45
            self.pub_playID.publish(play_id)
            
    def TagDetect(self,color_frame,depth_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        result_image = np.copy(rgb_image)
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        depth_to_color_image = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=1.0), cv.COLORMAP_JET)
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
        tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
        draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
        key = cv2.waitKey(10)

        if len(tags) > 0 :
            for i in range(len(tags)):
                center_x, center_y = tags[i].center
                pos = AprilTagInfo()
                pos.id = tags[i].tag_id
                pos.x = center_x
                pos.y = center_y
                pos.z = depth_image_info[int(center_y),int(center_x)]/1000
                cv.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
                print("center_x, center_y: ",center_x, center_y)
                print("depth_orin: ",depth_image_info[257,329]/1000)
                print("depth: ",depth_image_info[int(center_y),int(center_x)]/1000)
                if self.pubPos_flag == True:
                    if pos.z>0:
                        self.pos_info_pub.publish(pos)
                    else:
                        print("Invalid distance.")
                           
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("result_image", result_image)
        cv2.imshow("depth_image", depth_to_color_image)
        key = cv2.waitKey(1)





    def pub_arm(self, joints, id=6, angle=180.0, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        if len(joints) != 0: arm_joint.joints = joints
        else: arm_joint.joints = []
        self.pubPoint.publish(arm_joint)
        
    def GraspStatusCallback(self,msg):
        print("**")
        if msg.data == True:
            self.pubPos_flag = True

def main(args=None):
    rclpy.init(args=args)
    tag_detect = AprilTagDetectNode()
    tag_detect.pub_arm(tag_detect.init_joints)
    try:    
        rclpy.spin(tag_detect)
    except KeyboardInterrupt:
        pass
    finally:
        tag_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()