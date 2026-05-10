#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
import cv2
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32,Bool,Int8
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv
import time
import math

from message_filters import ApproximateTimeSynchronizer

from dofbot_pro_interface.msg import *      
from dofbot_pro_interface.srv import *

import transforms3d as tfs
import tf_transformations as tf         # ROS2使用tf_transformations
import threading

from ament_index_python import get_package_share_directory
import yaml
import os
from Arm_Lib import Arm_Device

pkg_path = get_package_share_directory('dofbot_pro_driver')
offset_file = os.path.join(pkg_path,'config', 'offset_value.yaml')

with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))
encoding = ['16UC1', '32FC1']

class Yolov11GraspNode(Node):
    def __init__(self):
        super().__init__('yolov11_grap')
        self.cx = 0
        self.cy = 0
        self.Arm = Arm_Device()
        self.sub_joint5 = self.create_subscription(Float32, "adjust_joint5",self.get_joint5Callback,qos_profile=1)
         # 发布器修改
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", qos_profile=10 )
        self.pubGraspStatus = self.create_publisher(Bool, "grasp_done", qos_profile=10)
        self.pub_playID = self.create_publisher(Int8, "player_id", qos_profile=10)
        # 其他订阅器
        self.subDetect = self.create_subscription(Yolov11Detect, "Yolov11DetectInfo",self.getDetectInfoCallback,qos_profile=10)
        self.depth_image_sub = self.create_subscription(Image,'/camera/depth/image_raw',self.getDepthCallback,qos_profile=1)
        self.sub_SortFlag = self.create_subscription(Bool,'sort_flag',self.getSortFlagCallback,qos_profile=10)
        # 服务客户端修改
        self.client = self.create_client(Kinemarics, "dofbot_kinemarics")

        self.color_x = 0.0
        self.color_y = 0.0
        self.color_z = 0.15
        self.grasp_flag = True
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 90.0]
        self.down_joint = [130.0, 55.0, 34.0, 16.0, 90.0,125.0]
        self.set_joint = [90.0, 120.0, 0.0, 0.0, 90.0, 90.0]
        self.gripper_joint = 90.0
        self.depth_bridge = CvBridge()
        self.start_sort = False
        self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.90000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,4.90000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.get_current_end_pos()

        self.name = None

        self.x_offset = offset_config.get('x_offset')
        self.y_offset = offset_config.get('y_offset')
        self.z_offset = offset_config.get('z_offset')
        
        
        self.play_id = Int8()
        self.recyclable_waste=['Newspaper','Zip_top_can','Book','Old_school_bag']
        self.toxic_waste=['Syringe','Expired_cosmetics','Used_batteries','Expired_tablets']
        self.wet_waste=['Fish_bone','Egg_shell','Apple_core','Watermelon_rind']
        self.dry_waste=['Toilet_paper','Peach_pit','Cigarette_butts','Disposable_chopsticks']
        print("Current_End_Pose: ",self.CurEndPos)
        print("Init Done")  

    def getDetectInfoCallback(self,msg):
        self.cx = int(msg.centerx)
        self.cy = int(msg.centery)
        self.name = msg.result
    
    def getDepthCallback(self,msg):
        depth_image = self.depth_bridge.imgmsg_to_cv2(msg, encoding[1])
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        if self.cy!=0 and self.cx!=0:
            self.dist = depth_image_info[self.cy,self.cx]/1000
            # print("self.dist",self.dist)
            # print("get the cx,cy",self.cx,self.cy)
            # print("get the detect result",self.name)
        
            if self.dist!=0 and self.name!=None:
                if self.start_sort == True:
                    camera_location = self.pixel_to_camera_depth((self.cx,self.cy),self.dist)
                    PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
                    #PoseEndMat = np.matmul(self.xyz_euler_to_mat(camera_location, (0, 0, 0)),self.EndToCamMat)
                    EndPointMat = self.get_end_point_mat()
                    WorldPose = np.matmul(EndPointMat, PoseEndMat) 
                    #WorldPose = np.matmul(PoseEndMat,EndPointMat)
                    pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
                    pose_T[0] = pose_T[0] + self.x_offset
                    pose_T[1] = pose_T[1] + self.y_offset
                    pose_T[2] = pose_T[2] + self.z_offset
                    threading.Thread(target=self.grasp, args=(pose_T,)).start()
                    print("get the detect result",self.name)
                    self.start_sort = False

    def getSortFlagCallback(self,msg):
        
        if msg.data == True:
            self.start_sort = True
            print(self.start_sort)

    def get_current_end_pos(self):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'dofbot_kinematics' not available!")
            return
        request = Kinemarics.Request()
        request.cur_joint1 = self.init_joints[0]
        request.cur_joint2 = self.init_joints[1]
        request.cur_joint3 = self.init_joints[2]
        request.cur_joint4 = self.init_joints[3]
        request.cur_joint5 = self.init_joints[4]
        request.kin_name = "fk"

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0) 
        response = future.result()
        # print(response)
        if isinstance(response, Kinemarics.Response):
            self.CurEndPos[0] = response.x
            self.CurEndPos[1] = response.y
            self.CurEndPos[2] = response.z
            self.CurEndPos[3] = response.roll
            self.CurEndPos[4] = response.pitch
            self.CurEndPos[5] = response.yaw

    def get_joint5Callback(self,msg):
        self.gripper_joint = msg.data

    def grasp(self,pose_T):
        print("------------------------------------------------")
        print("pose_T: ",pose_T)

        request = Kinemarics.Request()
        request.tar_x = pose_T[0] 
        request.tar_y = pose_T[1] 
        request.tar_z = pose_T[2] +  (math.sqrt(request.tar_y**2+request.tar_x**2)-0.181)*0.2
        request.kin_name = "ik"
        request.roll = self.CurEndPos[3]

        
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) 
            response = future.result()

            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = response.joint2 
            joints[2] = response.joint3 
            if response.joint4>90:
                joints[3] = 90 
            else:
                joints[3] = response.joint4 
            joints[4] = 90 
            joints[5] = 30

            self.pubTargetArm(joints)
            time.sleep(3.5)
            self.move()

        except Exception:
           pass

    def move(self):

        self.Arm.Arm_serial_servo_write(5, self.gripper_joint, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write(6, 140, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write(2, 120, 2000)
        time.sleep(1.5)
        print("name",self.name)

        if self.name in self.recyclable_waste:
            print("This is recyclable_waste.")
            if self.name == "Newspaper":
                self.play_id.data = 2
                self.pub_playID.publish(self.play_id)
            if self.name == "Zip_top_can":                
                self.play_id.data = 3
                self.pub_playID.publish(self.play_id)
            if self.name == "Book":                
                self.play_id.data = 4
                self.pub_playID.publish(self.play_id)
            if self.name == "Old_school_bag":
                
                self.play_id.data = 5
                self.pub_playID.publish(self.play_id)
                
            
            self.set_joint = [140, 20, 90, 3, 50.0,140]

        elif self.name in self.wet_waste:
            print("This is wet_waste.")
            if self.name == "Fish_bone":
                self.play_id.data = 6
                self.pub_playID.publish(self.play_id)   
            if self.name == "Watermelon_rind":
                self.play_id.data = 7
                self.pub_playID.publish(self.play_id)
            if self.name == "Apple_core":
                self.play_id.data = 8
                self.pub_playID.publish(self.play_id)
            if self.name == "Egg_shell":
                self.play_id.data = 9
                self.pub_playID.publish(self.play_id)          
            
            self.set_joint = [165, 38, 60, 2, 90.0,140]
            
        elif self.name in self.toxic_waste:
            print("This is toxic_waste.")
            if self.name == "Syringe":
                self.play_id.data = 10
                self.pub_playID.publish(self.play_id)   
            if self.name == "Expired_cosmetics":
                self.play_id.data = 11
                self.pub_playID.publish(self.play_id)
            if self.name == "Expired_tablets":
                self.play_id.data = 12
                self.pub_playID.publish(self.play_id)
            if self.name == "Used_batteries":
                self.play_id.data = 13
                self.pub_playID.publish(self.play_id)
            
            self.set_joint = [38, 20, 90, 2, 90.0,140]            
                 
        elif self.name in self.dry_waste:
            print("This is dry_waste.")
            if self.name == "Toilet_paper":
                self.play_id.data = 14
                self.pub_playID.publish(self.play_id)   
            if self.name == "Disposable_chopsticks":
                self.play_id.data = 15
                self.pub_playID.publish(self.play_id)
            if self.name == "Cigarette_butts":
                self.play_id.data = 16
                self.pub_playID.publish(self.play_id)
            if self.name == "Peach_pit":
                self.play_id.data = 17
                self.pub_playID.publish(self.play_id) 

            self.set_joint = [12, 38, 60, 0, 90.0,140]


        self.Arm.Arm_serial_servo_write6_array(self.set_joint, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write(6, 90, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write(2, 90, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write6_array(self.init_joints, 2000)
        print("Grasp done!")
        time.sleep(5.0)
        self.grasp_flag = True
        grasp_done = Bool()
        grasp_done.data = True
        self.pubGraspStatus.publish(grasp_done)
        self.name = None
        self.cx = 0
        self.cy = 0

    def get_end_point_mat(self):
        print("Get the current pose is ",self.CurEndPos)
        end_w,end_x,end_y,end_z = self.euler_to_quaternion(self.CurEndPos[3],self.CurEndPos[4],self.CurEndPos[5])
        endpoint_mat = self.xyz_quat_to_mat([self.CurEndPos[0],self.CurEndPos[1],self.CurEndPos[2]],[end_w,end_x,end_y,end_z])
        print("endpoint_mat: ",endpoint_mat)
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

    def pubTargetArm(self, joints, id=6, angle=180.0, runtime=2000):
        print(joints)
        self.Arm.Arm_serial_servo_write6(joints[0],joints[1],joints[2],joints[3],joints[4],joints[5],2000)
        
    def pubArm(self, joints, id=1, angle=90.0, run_time=2000):
        armjoint = ArmJoint()
        armjoint.run_time = run_time
        if len(joints) != 0: armjoint.joints = joints
        else:
            armjoint.id = id
            armjoint.angle = angle
        self.pubPoint.publish(armjoint)  

def main(args=None):
    rclpy.init(args=args)
    yolov11_grasp = Yolov11GraspNode()
    yolov11_grasp.pubArm(yolov11_grasp.init_joints)
    try:
        rclpy.spin(yolov11_grasp)
    except KeyboardInterrupt:
        pass
    finally:
        yolov11_grasp.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
