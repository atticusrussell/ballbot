#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32,Bool
import time
import threading
import numpy as np
import math
import os
from ament_index_python import get_package_share_directory
import transforms3d as tfs
import tf_transformations as tf  
from dofbot_pro_interface.msg import *
from dofbot_pro_interface.srv import *
import dofbot_pro_apriltag.PID as PID 
import yaml
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
class DofbotTrack(Node):
    def __init__(self):
        super().__init__('DofbotTrack')
        self.pubPoint = self.create_publisher(ArmJoint, 'TargetAngle',1)
        self.pub_buzzer = self.create_publisher(Bool,"Buzzer", 1)
        self.pubGrab = self.create_publisher(Bool,"grab", 1)

        self.Arm = Arm_Device()
        self.xservo_pid = PID.PositionalPID(0.25, 0.05, 0.03)
        self.yservo_pid = PID.PositionalPID(0.25, 0.1, 0.05)
        self.target_servox=90.0
        self.target_servoy=180.0

        self.a = 0
        self.b = 0
        self.c = 0
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        
        self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.down_joint = [130.0, 55.0, 34.0, 16.0, 90.0,125.0]
        self.move_time = 500.0
        self.dist_joint1 = 0.0
        self.dist_joint2 = 0.0
        self.dist_joint3 = 0.0
        self.dist_joint4 = 0.0
        self.init_flag = 0
        self.cx = 640.0
        self.cy = 480.0
        self.px = 0
        self.py = 0
        self.stamp_time = time.time()
        self.move_xy = True
        self.identify_ap = False
        self.depth_dist = 0.0
        self.distance = 0.0
        self.init_joints = [90.0, 150.0, 12.0, 20.0, 90.0, 30.0]
        self.y_out_range = False
        self.x_out_range = False
        self.joint2 = 150.0
        self.joint3 = 10.0
        self.joint4 = 20.0
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.0000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,5.00000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        
        
        self.get_current_end_pos(self.init_joints) 
        print("Init_Cur_Pose: ",self.CurEndPos)
        self.cur_joints = self.init_joints
        self.move_flag = True
        
        self.last_y = 0.28
        self.last_cur_y = 0
        self.XY_move = True
        self.joint1 = 90
        self.move_done = True
        
        self.compute_x = 0.0
        self.compute_y = 0.0
        self.set_joint5 = 90.0
        
        self.x_offset = offset_config.get('x_offset')
        self.y_offset = offset_config.get('y_offset')
        self.z_offset = offset_config.get('z_offset')
            
    def compute_joint1(self,center_x):
        #self.pub_arm(self.init_joint)
        self.px = center_x

        if not (self.target_servox>=180 and center_x<=320 and self.a == 1 or self.target_servox<=0 and center_x>=320 and self.a == 1):
            if(self.a == 0):
                
                self.xservo_pid.SystemOutput = center_x
                if self.x_out_range == True:
                    if self.target_servox<0:
                        self.target_servox = 0
                        self.xservo_pid.SetStepSignal(630)
                    if self.target_servox>0:
                        self.target_servox = 180
                        self.xservo_pid.SetStepSignal(10)
                    self.x_out_range = False
                else:
                    self.xservo_pid.SetStepSignal(320)
                    self.x_out_range = False
               
                self.xservo_pid.SetInertiaTime(0.01, 0.1)
                
                target_valuex = int(1500 + self.xservo_pid.SystemOutput)
                
                self.target_servox = int((target_valuex - 500) / 10) -16
              
                if self.target_servox > 180:
                    self.x_out_range = True
                    
                if self.target_servox < 0:
                    self.x_out_range = True
        self.joint1 = self.target_servox

    def compute_xy(self,x,y):
        c = math.sqrt(x**2 + y**2)
        angle = angle_rad = math.atan(x/y)
        
        print("d: ",c)
        print("angle_deg: ",angle)
        return c,angle
    
    def Depth_track(self,x,y,z):
        self.cur_depth = z
        self.get_current_end_pos(self.cur_joints)
        print("len: ",math.sqrt(self.CurEndPos[0]**2 + self.CurEndPos[1]**2+self.CurEndPos[2]**2))
        camera_location = self.pixel_to_camera_depth((x,y),z)
        PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
        EndPointMat = self.get_end_point_mat()
        WorldPose = np.matmul(EndPointMat, PoseEndMat) 
        pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
        c,rad = self.compute_xy(pose_T[0],pose_T[1])
        sin_value = math.sin(rad)
        cos_value = math.cos(rad)
         
        self.compute_x = (c-0.28)*sin_value
        self.compute_y = (c-0.28)*cos_value
        tan = self.compute_y/self.compute_x
        print("self.compute_x: ",self.compute_x)
        print("self.compute_y: ",self.compute_y)
        print("pose_T: ",pose_T)

        if pose_T[1]<0.28:
            print("Too close.")
            self.move_done = True
            self.move_flag = True

            self.adjust_joint1(pose_T)
        else:
            if self.move_flag==True:
                self.move_flag = False
                target_y =  pose_T[1] - self.last_y

                if (abs( pose_T[1] - self.CurEndPos[1])>0.28) or self.cur_depth<0.30:
                    print("9999999999999999999999999999999999999999")
                    print("depth: ",z)
                    print("x: ",x)
                    print("y: ",y)

                    self.last_y = pose_T[1]
                    grasp = threading.Thread(target=self.grasp, args=(pose_T,target_y,))
                    grasp.start()
                    grasp.join()
                else:
                    self.last_y = pose_T[1]
                    print("-------------------------------")
                    self.move_flag = True
                    self.move_done = True
          
    def adjust_joint1(self,pose_T):
        request = Kinemarics.Request()
        request.tar_x =  self.compute_x
        request.tar_y =  self.compute_y
        request.tar_z =  pose_T[2]
        request.roll = self.CurEndPos[3]
        request.kin_name = "ik"
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0) 
            response = future.result()
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = 150.0 #response.joint1
            joints[2] = 12.0 #response.joint1
            joints[3] = 20.0 #response.joint1
            joints[4] = 90.0 #response.joint1
            joints[5] = 30.0 #response.joint1
            self.cur_joints = joints
            self.pub_arm(joints)
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~121212")
        except Exception:
            rospy.loginfo("run error")
            
    def grasp(self,pose_T,target_y):

        request = Kinemarics.Request()
        if pose_T[0]<0:
            adjust = -0.03
        else:
            adjust = 0.07
        request.tar_x =  self.compute_x
        request.tar_y =  self.compute_y #target_y + self.CurEndPos[1]

        if pose_T[2]>0.30:
            pose_T[2] = 0.30
        request.tar_z =  pose_T[2]
        request.kin_name = "ik"
        request.roll = self.CurEndPos[3]

        print("calcutelate_request: ",request)
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0) 
            response = future.result()

            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = response.joint2
            joints[2] = response.joint3

            if response.joint4>90:
                joints[3] = 90.0
            else:
                joints[3] = response.joint4
            joints[4] = 90.0
            joints[5] = 30.0
            print("compute_joints: ",joints)
            if pose_T[1]>0.50:
                joints[3] = 65
            self.cur_joints = joints
            
            if request.tar_y<0.09:
                print("Back to init pose.")
                self.adjust_joint1(pose_T)

            else:
                self.pub_arm(joints)
            time.sleep(1.5)
            self.move_done = True
            self.move_flag = True
        except Exception:
           rospy.loginfo("run error")
                
    def get_current_end_pos(self,input_joints):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'dofbot_kinematics' not available!")
            return
        request = Kinemarics.Request()
        request.cur_joint1 = input_joints[0]
        request.cur_joint2 = input_joints[1]
        request.cur_joint3 = input_joints[2]
        request.cur_joint4 = input_joints[3]
        request.cur_joint5 = input_joints[4]
        request.kin_name = "fk"

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0) 
        response = future.result()
        if isinstance(response, Kinemarics.Response):
            self.CurEndPos[0] = response.x
            self.CurEndPos[1] = response.y
            self.CurEndPos[2] = response.z
            self.CurEndPos[3] = response.roll
            self.CurEndPos[4] = response.pitch
            self.CurEndPos[5] = response.yaw

    
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
    
    
    def pixel_to_camera_depth(self,pixel_coords, depth):
        fx, fy, cx, cy = self.camera_info_K[0],self.camera_info_K[4],self.camera_info_K[2],self.camera_info_K[5]
        px, py = pixel_coords
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth
        return np.array([x, y, z])    

    
    def get_end_point_mat(self):
        end_w,end_x,end_y,end_z = self.euler_to_quaternion(self.CurEndPos[3],self.CurEndPos[4],self.CurEndPos[5])
        endpoint_mat = self.xyz_quat_to_mat([self.CurEndPos[0],self.CurEndPos[1],self.CurEndPos[2]],[end_w,end_x,end_y,end_z])
        return endpoint_mat
    
    

    def pub_arm(self, joints, id=6, angle=180.0, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        arm_joint.joints = joints
        self.pubPoint.publish(arm_joint)

    def XY_track(self,center_x,center_y):
        #self.pub_arm(self.init_joint)
        self.px = center_x
        self.py = center_y

        if not (self.target_servox>=180 and center_x<=320 and self.a == 1 or self.target_servox<=0 and center_x>=320 and self.a == 1):
            if(self.a == 0):
                
                self.xservo_pid.SystemOutput = center_x
                if self.x_out_range == True:
                    if self.target_servox<0:
                        self.target_servox = 0
                        self.xservo_pid.SetStepSignal(630)
                    if self.target_servox>0:
                        self.target_servox = 180
                        self.xservo_pid.SetStepSignal(10)
                    self.x_out_range = False
                else:
                    self.xservo_pid.SetStepSignal(320)
                    self.x_out_range = False
               
                self.xservo_pid.SetInertiaTime(0.01, 0.1)
                
                target_valuex = int(1500 + self.xservo_pid.SystemOutput)
                
                self.target_servox = int((target_valuex - 500) / 10) -10
        
                if self.target_servox > 180:
                    self.x_out_range = True
                    
                if self.target_servox < 0:
                    self.x_out_range = True
                 
        #180 240 0 240            
        if not (self.target_servoy>=180 and center_y<=240 and self.b == 1 or self.target_servoy<=0 and center_y>=240 and self.b == 1):
            if(self.b == 0):
                self.yservo_pid.SystemOutput = center_y

                if self.y_out_range == True:
                    self.yservo_pid.SetStepSignal(450)
                    self.y_out_range = False
                else:
                    self.yservo_pid.SetStepSignal(240)

                self.yservo_pid.SetInertiaTime(0.01, 0.1)
               
                target_valuey = int(1500 + self.yservo_pid.SystemOutput)
                
                if target_valuey<=1000:
                    target_valuey = 1000
                    self.y_out_range = True
                self.target_servoy = int((target_valuey - 500) / 10) - 55#int((target_valuey - 500) / 10) - 55
                if self.target_servoy > 180: self.target_servoy = 180 #if self.target_servoy > 390: self.target_servoy = 390
                if self.target_servoy < 0: self.target_servoy = 0 

                joint2 = 120 + self.target_servoy
                joint3 =  self.target_servoy / 4.5
                joint4 =  self.target_servoy / 3
                

        
        joints_0 = [float(self.target_servox/1), float(joint2), float(joint3), float(joint4), 90.0, 30.0]

        self.Arm.Arm_serial_servo_write6_array(joints_0,2000)
        self.cur_joints = joints_0
    

    def Clamping(self,cx,cy,cz):
        self.get_current_end_pos(self.cur_joints)
        
        camera_location = self.pixel_to_camera_depth((cx,cy),cz)
        PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
        EndPointMat = self.get_end_point_mat()
        WorldPose = np.matmul(EndPointMat, PoseEndMat) 
        pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
        pose_T[0] = pose_T[0] + self.x_offset
        pose_T[1] = pose_T[1] + self.y_offset
        pose_T[2] = pose_T[2] + self.z_offset
        print("pose_T: ",pose_T)
        request = Kinemarics.Request()
        request.tar_x = pose_T[0] 
        request.tar_y = pose_T[1]
        request.tar_z = pose_T[2]  + (math.sqrt(request.tar_y**2+request.tar_x**2)-0.181)*0.2
        request.kin_name = "ik"
        request.roll = self.CurEndPos[3] 
        print("calcutelate_request: ",request)
        
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0) 
            response = future.result()
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = response.joint2
            joints[2] = response.joint3
            if response.joint4>90:
                joints[3] = 90.0
            else:
                joints[3] = response.joint4
            joints[4] = 90.0
            joints[5] = 30.0
            print("compute_joints: ",joints)
            dist = math.sqrt(request.tar_y ** 2 + request.tar_x** 2)
            if dist>0.18 and dist<0.30:
                self.Buzzer()
                print("Clamp Mode.")
                self.pub_arm(joints)
                time.sleep(3.5)
                self.move()
            else:
                print("Too far or too close.")

        except Exception:
           pass
            
    def move(self):
        print("set_joint5: ",self.set_joint5)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write(5, self.set_joint5, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write(6, 125.0, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write(2, 120.0, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write6_array(self.down_joint,2000)
        time.sleep(3)
        self.Arm.Arm_serial_servo_write(6, 90.0, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write(2, 90.0, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)

          
    def Buzzer(self):
        beep = Bool()
        beep.data = True
        self.pub_buzzer.publish(beep)
        time.sleep(1)
        beep.data = False
        self.pub_buzzer.publish(beep)
        time.sleep(1)


