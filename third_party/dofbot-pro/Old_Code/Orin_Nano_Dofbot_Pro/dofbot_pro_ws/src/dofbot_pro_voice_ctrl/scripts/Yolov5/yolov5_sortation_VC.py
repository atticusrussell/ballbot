#!/usr/bin/env python
# -*- coding: utf-8 -*-
print("语音指令词如下：")
print("------开始垃圾分拣------")

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32,Bool,Int8
from cv_bridge import CvBridge
import time
import math
import rospkg
from dofbot_pro_info.msg import *
from dofbot_pro_info.srv import *
import transforms3d as tfs
import tf.transformations as tf
import threading
encoding = ['16UC1', '32FC1']
import yaml
offset_file = rospkg.RosPack().get_path("dofbot_pro_info") + "/param/offset_value.yaml"
with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))
class Yolov5GraspNode:
    def __init__(self):
        nodeName = 'yolov5_grap'
        rospy.init_node(nodeName)
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.pubGraspStatus = rospy.Publisher("grasp_done", Bool, queue_size=1)
        self.pub_playID = rospy.Publisher("player_id", Int8, queue_size=1)
        self.subDetect = rospy.Subscriber("Yolov5DetectInfo", Yolov5Detect, self.getDetectInfoCallback)
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_raw',Image,self.getDepthCallback)
        self.sub_SortFlag = rospy.Subscriber('sort_flag',Bool,self.getSortFlagCallback)
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)
        self.init_joints = [90.0, 120, 0.0, 0.0, 90, 90]
        self.down_joint = [130.0, 55.0, 34.0, 16.0, 90.0,135]
        self.set_joint = [90.0, 120, 0.0, 0.0, 90, 90]
        self.gripper_joint = 90
        self.depth_bridge = CvBridge()
        self.start_sort = True
        self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.90000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,4.90000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.get_current_end_pos()
        self.cx = 0
        self.cy = 0
        self.name = None
        self.play_id = Int8()
        self.recyclable_waste=['Newspaper','Zip_top_can','Book','Old_school_bag']
        self.toxic_waste=['Syringe','Expired_cosmetics','Used_batteries','Expired_tablets']
        self.wet_waste=['Fish_bone','Egg_shell','Apple_core','Watermelon_rind']
        self.dry_waste=['Toilet_paper','Peach_pit','Cigarette_butts','Disposable_chopsticks']
        self.x_offset = offset_config.get('x_offset')
        self.y_offset = offset_config.get('y_offset')
        self.z_offset = offset_config.get('z_offset')
        print("Current_End_Pose: ",self.CurEndPos)
        print("Init Done")     
        
    def getDetectInfoCallback(self,msg):
        self.cx = int(msg.centerx)
        self.cy = int(msg.centery)
        self.name = msg.result
    
    def getDepthCallback(self,msg):
        depth_image = self.depth_bridge.imgmsg_to_cv2(msg, encoding[1])
        frame = cv2.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        if self.cy!=0 and self.cx!=0:
            self.dist = depth_image_info[self.cy,self.cx]/1000
            print("self.dist",self.dist)
            print("get the cx,cy",self.cx,self.cy)
            print("get the detect result",self.name)
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
                    grasp = threading.Thread(target=self.grasp, args=(pose_T,))
                    grasp.start()
                    grasp.join()
            
    
    def getSortFlagCallback(self,msg):
        if msg.data == True:
            self.start_sort = True
            
        
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
            time.sleep(2.5)
            self.move()

        except Exception:
           rospy.loginfo("run error")

    def move(self):
        print("self.gripper_joint = ",self.gripper_joint)
        print("name: ",self.name)
        self.pubArm([],5, self.gripper_joint, 2000)
        time.sleep(2.5)
        self.pubArm([],6, 135, 2000)
        time.sleep(2.5)
        self.pubArm([],2, 120, 2000)
        time.sleep(1.5)
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
                
            
            self.set_joint = [140, 20, 90, 3, 90.0,135]
           

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
            
            self.set_joint = [165, 38, 60, 2, 90.0,135]
            
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
            
            self.set_joint = [38, 20, 90, 2, 90.0,135]           
                 
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
            
            self.set_joint = [12, 38, 60, 0, 90.0,135]
        self.pubArm(self.set_joint)
        time.sleep(2.5)
        self.pubArm([],6, 90, 2000)
        time.sleep(2.5)
        self.pubArm([],2, 90, 2000)
        time.sleep(2.5)
        self.pubArm(self.init_joints)
        print("Grasp done!")
        time.sleep(5.0)
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
        yolov5_grasp = Yolov5GraspNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

