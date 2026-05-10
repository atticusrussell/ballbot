#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from std_msgs.msg import Float32,Bool
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
encoding = ['16UC1', '32FC1']
class ComputeWidthNode:
    def __init__(self):
        nodeName = 'computewidth'
        rospy.init_node(nodeName)

        self.sub_joint5 = rospy.Subscriber("adjust_joint5",Float32,self.get_joint5Callback)
        self.sub_width = rospy.Subscriber("width_info",WidthInfo,self.get_widthCallback)
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_raw',Image)
        self.TimeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.depth_sub],10,0.5,allow_headerless=True)
        self.TimeSynchronizer.registerCallback(self.GetDepthInfo)
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.pubGraspStatus = rospy.Publisher("grasp_done", Bool, queue_size=1)
        self.pubJoint6 = rospy.Publisher("joint6", Float32, queue_size=1)
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)
        self.color_x = 480.0
        self.color_y = 320.0
        self.color_z = 0.15
        self.depth_bridge = CvBridge()
        self.lx = self.ly = self.rx = self.ry = 0.0
        self.grasp_flag = True
        self.init_joints = [90.0, 150, 12.0, 10.0, 90, 90]
        self.down_joint = [130.0, 55.0, 34.0, 16.0, 90.0,125]
        self.gripper_joint = 90
        self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.00000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,5.50000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.get_current_end_pos()
        self.cur_tagId = 0
        
        
        print("Current_End_Pose: ",self.CurEndPos)
        print("Init Done")     
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
            self.CurEndPos[0] = response.x
            self.CurEndPos[1] = response.y
            self.CurEndPos[2] = response.z
            self.CurEndPos[3] = response.Roll
            self.CurEndPos[4] = response.Pitch
            self.CurEndPos[5] = response.Yaw

    def get_joint5Callback(self,msg):
        self.gripper_joint = msg.data
        
    def get_widthCallback(self,msg):
        self.lx = msg.L_x
        self.ly = msg.L_y
        self.rx = msg.R_x
        self.ry = msg.R_y
        
    def GetDepthInfo(self,depth_frame):
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        if self.lx!=0.0:
            depth_l = depth_image_info[int(self.lx),int(self.ly)]/1000
            depth_r = depth_image_info[int(self.rx),int(self.ry)]/1000
            #print("depth_l: ",depth_l)
            #print("depth_r: ",depth_r)
            if depth_l!=0 and depth_r!=0:
                l_pos = self.CalculateWidth(self.lx,self.ly,depth_l)
                r_pos = self.CalculateWidth(self.rx,self.ry,depth_r)
                #print("l_pos: ",l_pos[0])
                #print("r_pos: ",r_pos[0])
                res_width = round(abs(r_pos[0] - l_pos[0]),2)
                print("res_width: ",res_width)
                joint6_angle = Float32()
                joint6 = 180 - res_width*100*17
                joint6_angle.data = joint6
                self.pubJoint6.publish(joint6_angle)
                print("joint6: ",joint6)



    #颜色信息的回调函数，包括中心xy坐标和深度值z
    def CalculateWidth(self,x,y,z):
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

     
        
if __name__ == '__main__':
    try:
        compute_width = ComputeWidthNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

