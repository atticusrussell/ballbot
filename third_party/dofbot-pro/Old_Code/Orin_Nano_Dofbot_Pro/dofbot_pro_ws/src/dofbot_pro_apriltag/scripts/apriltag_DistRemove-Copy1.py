#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from std_msgs.msg import Float32,Bool,Int8
from cv_bridge import CvBridge
import cv2 as cv
import time
import rospkg
import math
from dofbot_pro_info.msg import *
from dofbot_pro_info.srv import *
import transforms3d as tfs
import tf.transformations as tf
import threading
class TagGraspNode:
    def __init__(self):
        nodeName = 'color_grap'
        rospy.init_node(nodeName)
        self.tag_info_sub = rospy.Subscriber("TagInfo", AprilTagInfo, self.tag_info_callback, queue_size=1)
        self.sub_joint5 = rospy.Subscriber("adjust_joint5",Float32,self.get_joint5Callback)
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.pubGraspStatus = rospy.Publisher("grasp_done", Bool, queue_size=1)
        self.pub_heigh = rospy.Publisher("compute_heigh", Float32, queue_size=1)
        self.pub_buzzer = rospy.Publisher("Buzzer", Bool, queue_size=1)
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)
        self.sub_targetID = rospy.Subscriber("TargetId",Int8,self.GetTargetIDCallback, queue_size=1)
        self.color_x = 480.0
        self.color_y = 320.0
        self.color_z = 0.15
        self.grasp_flag = True
        self.set_dist = 0
        self.set_height = 0
        self.init_joints = [90.0, 120, 0.0, 0.0, 90, 90]
        self.down_joint = [130.0, 55.0, 34.0, 16.0, 90.0,125]
        self.detect_joints = [90,150,12,20,90,30]
        self.search_joints = [90.0, 120, 0.0, 0.0, 90, 30]
        self.gripper_joint = 90
        self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.00000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,5.50000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.get_current_end_pos()
        self.cur_tagId = 0
        #print("Current_End_Pose: ",self.CurEndPos)
        print("Init Done")     
       
    def GetTargetIDCallback(self,msg):
        self.set_dist = 0.15 + msg.data*0.01
        self.pubArm(self.init_joints)
        time.sleep(2.0)
        
        
    def get_current_end_pos(self):
        self.client.wait_for_service()
        request = kinemaricsRequest()
        request.cur_joint1 = self.init_joints[0]
        request.cur_joint2 = self.init_joints[1]
        request.cur_joint3 = self.init_joints[2]
        request.cur_joint4 = self.init_joints[3]
        request.cur_joint5 = self.init_joints[4]
        request.kin_name = "fk"
        response = self.client.call(request)
        if isinstance(response, kinemaricsResponse):
            print(response)
            self.CurEndPos[0] = response.x
            self.CurEndPos[1] = response.y
            self.CurEndPos[2] = response.z
            self.CurEndPos[3] = response.Roll
            self.CurEndPos[4] = response.Pitch
            self.CurEndPos[5] = response.Yaw

    def get_joint5Callback(self,msg):
        self.gripper_joint = msg.data
        
            
    #颜色信息的回调函数，包括中心xy坐标和深度值z
    def tag_info_callback(self,msg):
        print("msg: ",msg)
        if self.set_dist == 0: return
        if self.set_dist != 0:
            time.sleep(3.0)
            pos_x = msg.x
            pos_y = msg.y
            pos_z = msg.z
            self.cur_tagId = msg.id
            
            if pos_z!=0.0 :
                print("xyz id : ",pos_x,pos_y,pos_z,self.cur_tagId)
                self.get_current_end_pos()
                camera_location = self.pixel_to_camera_depth((pos_x,pos_y),pos_z)
                #print("camera_location: ",camera_location)
                PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
                #PoseEndMat = np.matmul(self.xyz_euler_to_mat(camera_location, (0, 0, 0)),self.EndToCamMat)
                EndPointMat = self.get_end_point_mat()
                WorldPose = np.matmul(EndPointMat, PoseEndMat) 
                #WorldPose = np.matmul(PoseEndMat,EndPointMat)
                pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
                print("pose_T: ",pose_T)
                if self.grasp_flag == True and pose_T[1]<=self.set_dist :
                    print("Discover the matching apriltag.")
                    self.grasp_flag = False
                    grasp = threading.Thread(target=self.grasp, args=(pose_T,))
                    grasp.start()
                    grasp.join()
                elif (pose_T[1]>self.set_dist and self.set_dist != 0) :
                    print("There are no eligible apriltag.")
                    self.set_dist = 0
                    self.set_height = 0
                    self.shake()
                    time.sleep(2)
                    self.pubArm(self.detect_joints)
                    time.sleep(2)

 
    def grasp(self,pose_T):
        print("------------------------------------------------")
        print("pose_T: ",pose_T)
        request = kinemaricsRequest()
        request.tar_x = pose_T[0] + 0.002
        request.tar_y = pose_T[1]  - 0.005
        request.tar_z = pose_T[2] - request.tar_y*0.1
        request.kin_name = "ik"
        request.Roll = self.CurEndPos[3]


        print("calcutelate_request: ",request)
        try:
            response = self.client.call(request)
            #print("calcutelate_response: ",response)
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
            print("compute_joints: ",joints)
            self.pubTargetArm(joints)
            time.sleep(3.5)
            self.move()

        except Exception:
           rospy.loginfo("run error")

    def move(self):
        print("self.gripper_joint = ",self.gripper_joint)
        self.pubArm([],5, self.gripper_joint, 2000)
        time.sleep(2.5)
        self.pubArm([],6, 125, 2000)
        time.sleep(2.5)
        self.pubArm([],2, 90, 2000)
        time.sleep(2.5)
        '''self.Arm.Arm_Buzzer_On()
        time.sleep(1.5)
        self.Arm.Arm_Buzzer_Off()'''
        #self.pubArm([],1, 120, 2000)
        #time.sleep(2.5)
        if self.cur_tagId == 1:
            self.down_joint = [130.0, 55.0, 34.0, 16.0, 90.0,125]
        elif self.cur_tagId == 2:
            self.down_joint = [170.0, 55.0, 34.0, 16.0, 90.0,125]
        elif self.cur_tagId == 3:
            self.down_joint = [50.0, 55.0, 34.0, 16.0, 90.0,125]
        elif self.cur_tagId == 4:
            self.down_joint = [10.0, 55.0, 34.0, 16.0, 90.0,125]
        self.pubArm(self.down_joint)
        time.sleep(2.5)
        self.pubArm([],6, 90, 2000)
        time.sleep(2.5)
        self.pubArm(self.init_joints)
        self.grasp_flag = True
        #grasp_done = Bool()
        #grasp_done.data = True
        #self.pubGraspStatus.publish(grasp_done)
        #time.sleep(2.5)

    def shake(self):
        beep = Bool()
        beep.data = True
        self.pub_buzzer.publish(beep)
        time.sleep(1)
        beep.data = False
        self.pub_buzzer.publish(beep)
        time.sleep(1)
        self.pubArm([],1, 60, 800)
        time.sleep(0.5)
        self.pubArm([],1, 120, 800)
        time.sleep(0.5)   
        self.pubArm([],1, 60, 800)
        time.sleep(0.5)
        self.pubArm([],1, 120, 800)
        time.sleep(0.5) 
        self.pubArm([],1, 90, 800)
        time.sleep(0.5) 
        
        
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

    def pubTargetArm(self, joints, id=6, angle=180, runtime=2000):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        if len(joints) != 0: arm_joint.joints = joints
        else: arm_joint.joints = []
        self.pubPoint.publish(arm_joint)
        
    def pubArm(self, joints, id=1, angle=90, run_time=2000):
        armjoint = ArmJoint()
        armjoint.run_time = run_time
        if len(joints) != 0: armjoint.joints = joints
        else:
            armjoint.id = id
            armjoint.angle = angle
        self.pubPoint.publish(armjoint)      
        
if __name__ == '__main__':
    try:
        tag_grasp = TagGraspNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

