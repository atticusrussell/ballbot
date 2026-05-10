#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from std_msgs.msg import Float32,Bool,Int8
import time
import math
from dofbot_pro_info.msg import *
from dofbot_pro_info.srv import *
import transforms3d as tfs
import tf.transformations as tf
import threading
import yaml
import rospkg
offset_file = rospkg.RosPack().get_path("dofbot_pro_info") + "/param/offset_value.yaml"
with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))
class ColorGraspNode:
    def __init__(self):
        nodeName = 'color_grap'
        rospy.init_node(nodeName)
        self.Color_Pos_sub = rospy.Subscriber('xyz', Position, self.color_pos_callback, queue_size=1)
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.pubGraspStatus = rospy.Publisher("grasp_done", Bool, queue_size=1)
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)
        self.grasp_flag = True
        self.init_joints = [90.0, 120, 0.0, 0.0, 90, 90]
        self.down_joint = [150.0, 55.0, 34.0, 16.0, 90.0,135]
        self.gripper_joint = 90
        self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.90000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,4.90000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.get_current_end_pos()
        self.x_offset = offset_config.get('x_offset')
        self.y_offset = offset_config.get('y_offset')
        self.z_offset = offset_config.get('z_offset')
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


        
            
    #颜色信息的回调函数，包括中心xy坐标和深度值z
    def color_pos_callback(self,msg):
        print("msg: ",msg)
        color_x = msg.x
        color_y = msg.y
        color_z = msg.z
        if color_z!=0.0:
            camera_location = self.pixel_to_camera_depth((color_x,color_y),color_z)
            #print("camera_location: ",camera_location)
            PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
            #PoseEndMat = np.matmul(self.xyz_euler_to_mat(camera_location, (0, 0, 0)),self.EndToCamMat)
            EndPointMat = self.get_end_point_mat()
            WorldPose = np.matmul(EndPointMat, PoseEndMat) 
            #WorldPose = np.matmul(PoseEndMat,EndPointMat)
            pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
            pose_T[0] = pose_T[0] + self.x_offset
            pose_T[1] = pose_T[1] + self.y_offset
            pose_T[2] = pose_T[2] + self.z_offset
            #print("pose_T: ",pose_T)
            if self.grasp_flag == True:
                grasp = threading.Thread(target=self.grasp, args=(pose_T,))
                grasp.start()
                grasp.join()
                #self.grasp_flag = False
        
 
    def grasp(self,pose_T):
        print("------------------------------------------------")
        print("pose_T: ",pose_T)
        request = kinemaricsRequest()
        request.tar_x = pose_T[0] 
        request.tar_y = pose_T[1] 
        request.tar_z = pose_T[2] +  (math.sqrt(request.tar_y**2+request.tar_x**2)-0.181)*0.2 #0.2为比例系数，根据实际夹取效果进行调整
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
        self.pubArm([],6, 135, 2000)
        time.sleep(2.5)
        self.pubArm([],2, 120, 2000)
        time.sleep(2.5)
        '''self.Arm.Arm_Buzzer_On()
        time.sleep(1.5)
        self.Arm.Arm_Buzzer_Off()'''
        self.pubArm([],1, 150, 2000)
        time.sleep(2.5)
        self.pubArm(self.down_joint)
        time.sleep(2.5)
        self.pubArm([],6, 90, 2000)
        time.sleep(2.5)
        self.pubArm([],2, 90, 2000)
        time.sleep(2.5)
        self.pubArm(self.init_joints)
        time.sleep(2.5)
        self.grasp_flag = True
        grasp_done = Bool()
        grasp_done.data = True
        self.pubGraspStatus.publish(grasp_done)
        
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
        color_grasp = ColorGraspNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

