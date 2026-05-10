#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int8, Bool
from dt_apriltags import Detector
from dofbot_pro_apriltag.vutils import draw_tags
from cv_bridge import CvBridge
import cv2 as cv
from dofbot_pro_interface.srv import *  
from dofbot_pro_interface.msg import ArmJoint, AprilTagInfo
import pyzbar.pyzbar as pyzbar
import time
import queue
import math
import os
encoding = ['16UC1', '32FC1']
import threading
import transforms3d as tfs
import tf_transformations as tf   
from Arm_Lib import Arm_Device

class AprilTagDetectNode(Node):
    def __init__(self):
        super().__init__('apriltag_detect') 
        
        # 初始化参数
        self.detect_joints = [90.0, 150.0, 12.0, 20.0, 90.0, 30.0]
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 90.0]
        self.search_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
        self.Arm = Arm_Device()
        
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

        self.set_height = 0.0
        self.set_dist = 0.0 
        self.detect_flag = False
        self.done_flag = True
        
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
   
        self.pr_time = time.time()
        self.cnt = 0
        self.Center_x_list = []
        self.Center_y_list = []
        self.search_joints = [90.0,150.0,12.0,20.0,90.0,30.0]

        self.CurEndPos = [-0.006,0.116261662208,0.0911289015753,-1.04719,-0.0,0.0]
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.90000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,4.90000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
    


    def TagDetect(self,color_frame,depth_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        result_image = np.copy(rgb_image)
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1.0), cv2.COLORMAP_JET)
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
        tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
        draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
        key = cv2.waitKey(10)
        self.Center_x_list = list(range(len(tags)))
        self.Center_y_list = list(range(len(tags)))
        if key == 32:
            self.pubPos_flag = True
        if len(tags) > 0 and self.done_flag == True:
            for i in range(len(tags)):
                center_x, center_y = tags[i].center
                
                cv2.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
                self.Center_x_list[i] = center_x
                self.Center_y_list[i] = center_y
                cx = center_x
                cy = center_y
                cz = depth_image_info[int(cy),int(cx)]/1000
                pose = self.compute_heigh(cx,cy,cz)
                heigh_detect = round(pose[2],4)*1000 
                heigh = 'heigh: ' + str(heigh_detect) + 'mm'
                
                dist_detect = math.sqrt(pose[1] ** 2 + pose[0]** 2)
                dist_detect = round(dist_detect,3)*1000
                dist = 'dist: ' + str(dist_detect) + 'mm'
                cv.putText(result_image, str(self.set_height), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                cv.putText(result_image, heigh, (int(cx)+5, int(cy)-15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv.putText(result_image, dist, (int(cx)+5, int(cy)+15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                # print("Not high enough",heigh_detect)

                if heigh_detect>=self.set_height and self.set_height!=0 and  self.pubPos_flag == True:
                    print("self.set_height: ",self.set_height)
                    print("heigh_detect: ",heigh_detect)
                    self.detect_flag  = True
                    tag = AprilTagInfo()
                    tag.id = tags[i].tag_id
                    tag.x = self.Center_x_list[i]
                    tag.y = self.Center_y_list[i]
                    tag.z = depth_image_info[int(tag.y),int(tag.x)]/1000

                    print("tag_z: ",tag.z)
   
                    if tag.z!=0:
                        self.tag_info_pub.publish(tag)
                        self.pubPos_flag = False
                        self.done_flag = False
                    else:
                        print("Invalid distance.")

            if self.detect_flag != True and  self.set_height!=0 and self.pubPos_flag == True:
                print("----------------------------------------")
                self.set_height!=0
                self.shake()
                time.sleep(2)
                self.pub_arm(self.search_joints)
                grasp_done = Bool()
                grasp_done.data = True
                self.pubGraspStatus.publish(grasp_done)

            self.pubPos_flag = False 
        elif self.pubPos_flag == True and len(tags) == 0:
            self.shake()
            time.sleep(2)
            print("-----********---------------")
            self.Arm.Arm_serial_servo_write6_array(self.search_joints,2000)
            grasp_done = Bool()
            grasp_done.data = True
            self.pubGraspStatus.publish(grasp_done)
            self.pubPos_flag = False
            
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
        arm_joint.joints = joints
        self.pubPoint.publish(arm_joint)
        
    def GraspStatusCallback(self,msg):
        if msg.data == True:
            self.done_flag = True
            self.detect_flag  = False
            self.pubPos_flag = False
    
    def compute_heigh(self,x,y,z):
        camera_location = self.pixel_to_camera_depth((x,y),z)
        #print("camera_location: ",camera_location)
        PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
        #PoseEndMat = np.matmul(self.xyz_euler_to_mat(camera_location, (0, 0, 0)),self.EndToCamMat)
        EndPointMat = self.get_end_point_mat()
        WorldPose = np.matmul(EndPointMat, PoseEndMat) 
        #WorldPose = np.matmul(PoseEndMat,EndPointMat)
        pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)

        return pose_T
        

    def get_end_point_mat(self):
        #print("Get the current pose is ",self.CurEndPos)
        end_w,end_x,end_y,end_z = self.euler_to_quaternion(self.CurEndPos[3],self.CurEndPos[4],self.CurEndPos[5])
        endpoint_mat = self.xyz_quat_to_mat([self.CurEndPos[0],self.CurEndPos[1],self.CurEndPos[2]],[end_w,end_x,end_y,end_z])
        #print("endpoint_mat: ",endpoint_mat)
        return endpoint_mat
        
               
    
    #像素坐标转换到深度相机三维坐标坐标，也就是深度相机坐标系下的抓取点三维坐标
    def pixel_to_camera_depth(self,pixel_coords, depth):
        fx, fy, cx, cy = self.camera_info_K[0],self.camera_info_K[4],self.camera_info_K[2],self.camera_info_K[5]
        px, py = pixel_coords
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth
        return np.array([x, y, z])
    
    #通过平移向量和旋转的欧拉角得到变换矩阵    
    def xyz_euler_to_mat(self,xyz, euler, degrees=False):
        if degrees:
            mat = tfs.euler.euler2mat(math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]))
        else:
            mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
        mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
        return mat        
    
    #欧拉角转四元数
    def euler_to_quaternion(self,roll,pitch, yaw):
        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
        qw = quaternion[3]
        qx = quaternion[0]
        qy = quaternion[1]
        qz = quaternion[2]
        #print("quaternion: ",quaternion )
        return np.array([qw, qx, qy, qz])

    #通过平移向量和旋转的四元数得到变换矩阵
    def xyz_quat_to_mat(self,xyz, quat):
        mat = tfs.quaternions.quat2mat(np.asarray(quat))
        mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
        return mat

    #把旋转变换矩阵转换成平移向量和欧拉角
    def mat_to_xyz_euler(self,mat, degrees=False):
        t, r, _, _ = tfs.affines.decompose(mat)
        if degrees:
            euler = np.degrees(tfs.euler.mat2euler(r))
        else:
            euler = tfs.euler.mat2euler(r)
        return t, euler
        
        
    def shake(self):
        # beep = Bool()
        # beep.data = True
        # self.pub_buzzer.publish(beep)
        # time.sleep(1)
        # beep.data = False
        # self.pub_buzzer.publish(beep)
        self.pub_arm([],1, 60.0, 600)
        time.sleep(0.5)
        self.pub_arm([],1, 120.0, 600)
        time.sleep(0.5)   
        self.pub_arm([],1, 60.0, 600)
        time.sleep(0.5)
        self.pub_arm([],1, 120.0, 600)
        time.sleep(0.5) 
        self.pub_arm([],1, 90.0, 600)
        time.sleep(0.5)      
        
    def GetTargetIDCallback(self,msg):
        print("msg.data: ",msg.data)
        self.set_dist = 150 + msg.data*10
        self.set_height = 30 + msg.data*10
        print("self.set_height: ",self.set_height)

        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
        #time.sleep(3.0)        
        
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

