#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import PID
from yahboomcar_msgs.msg import *
import rospy
from std_msgs.msg import Float32,Bool
import time
import threading
import numpy as np
#Arm = Arm_Device()
from dofbot_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse
import transforms3d as tfs
import math
import transforms3d as tfs
import tf.transformations as tf
from dofbot_pro_info.msg import *
from dofbot_pro_info.srv import *
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
class DofbotTrack:
    def __init__(self):
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.pubGrab = rospy.Publisher("grab", Bool, queue_size=1)
        self.sub_move = rospy.Subscriber("move",Bool,self.moveCallBack)
        self.xservo_pid = PID.PositionalPID(0.25, 0.1, 0.05)
        self.yservo_pid = PID.PositionalPID(0.25, 0.1, 0.05)
        self.target_servox=90
        self.target_servoy=180
        self.init_joints=180
        self.a = 0
        self.b = 0
        self.c = 0
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)
        self.pub_buzzer = rospy.Publisher("Buzzer", Bool, queue_size=1)
        self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.down_joint = [130.0, 55.0, 34.0, 16.0, 90.0,120]
        self.move_time = 500
        self.dist_joint1 = 0.0
        self.dist_joint2 = 0.0
        self.dist_joint3 = 0.0
        self.dist_joint4 = 0.0
        self.init_flag = 0
        self.cx = 640.0
        self.cy = 480.0
        self.px = 0.0
        self.py = 0.0
        self.stamp_time = time.time()
        self.move_xy = True
        self.identify_ap = False
        self.depth_dist = 0
        self.distance = 0
        self.init_joints = [90.0, 150, 12, 20.0, 90, 30]
        self.y_out_range = False
        self.x_out_range = False
        self.joint2 = 150
        self.joint3 = 10
        self.joint4 = 20
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.00000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,4.90000000e-02],
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
        
    def moveCallBack(self,msg):
        if msg.data == True:
            self.move_flag = True
            
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
                #print("self.target_servox:",self.target_servox)
                
                if self.target_servox > 180:
                    self.x_out_range = True
                    
                if self.target_servox < 0:
                    self.x_out_range = True
        self.joint1 = self.target_servox
        #self.cur_joints[0] = self.joint1
        #print("compute joint1: ",self.joint1)
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
        #print("compute_x: ",self.compute_x)
       # print("compute_y: ",self.compute_y)
        #print("self.CurEndPos[3]: ",self.CurEndPos[3])
        '''print("self.CurEndPos[1]: ",self.CurEndPos[1])
        print("res: ",pose_T[1] - self.last_y)
        print("self.CurEndPos[1]: ",self.CurEndPos[1])'''
        if pose_T[1]<0.28:
            print("Too close.")
            self.move_done = True
            self.move_flag = True
            #self.cur_joints = self.init_joints
            #self.pub_arm(self.init_joints)
            #self.last_y = pose_T[1]
            self.adjust_joint1(pose_T)
        else:
            if self.move_flag==True:
                self.move_flag = False
                target_y =  pose_T[1] - self.last_y
                #print("cal_target_y",target_y)
                #print("abs: ",abs( pose_T[1] - self.CurEndPos[1]))
                #print("pose_T[1]: ",pose_T[1])
                #print("self.CurEndPos[1]: ",self.CurEndPos[1])
                if (abs( pose_T[1] - self.CurEndPos[1])>0.28) or self.cur_depth<0.30:
                    print("9999999999999999999999999999999999999999")
                    print("depth: ",z)
                    print("x: ",x)
                    print("y: ",y)
                    #print("pose_T[1]: ",pose_T[1])
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
        request = kinemaricsRequest()
        request.tar_x =  self.compute_x
        request.tar_y =  self.compute_y
        request.tar_z =  pose_T[2]
        request.Roll = self.CurEndPos[3]
        request.kin_name = "ik"
        try:
            response = self.client.call(request)
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = 150 #response.joint1
            joints[2] = 12 #response.joint1
            joints[3] = 20 #response.joint1
            joints[4] = 90 #response.joint1
            joints[5] = 30 #response.joint1
            self.cur_joints = joints
            self.pub_arm(joints)
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~121212")
        except Exception:
            rospy.loginfo("run error")
            
    def grasp(self,pose_T,target_y):
        #print("pose_T: ",pose_T)
        request = kinemaricsRequest()
        if pose_T[0]<0:
            adjust = -0.03
        else:
            adjust = 0.07
        request.tar_x =  self.compute_x
        request.tar_y =  self.compute_y #target_y + self.CurEndPos[1]
        #self.last_y = request.tar_y
        #print("")
        if pose_T[2]>0.30:
            pose_T[2] = 0.30
        request.tar_z =  pose_T[2]
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
            #if joints[3] <10:
                #joints[3] = 10
            joints[4] = 90 
            joints[5] = 30
            print("compute_joints: ",joints)
            if pose_T[1]>0.50:
                joints[3] = 65
            self.cur_joints = joints
            
            if request.tar_y<0.09:
                #self.pub_arm(self.init_joints)
                print("Back to init pose.")
                self.adjust_joint1(pose_T)
                #self.cur_joints = self.init_joints
            else:
                self.pub_arm(joints)
            time.sleep(1.5)
            self.move_done = True
            #time.sleep(10.0)
            self.move_flag = True
            #time.sleep(5.0)
            #self.move()

        except Exception:
           rospy.loginfo("run error")
                
    def get_current_end_pos(self,input_joints):
        self.client.wait_for_service()
        request = kinemaricsRequest()
        request.cur_joint1 = input_joints[0]
        request.cur_joint2 = input_joints[1]
        request.cur_joint3 = input_joints[2]
        request.cur_joint4 = input_joints[3]
        request.cur_joint5 = input_joints[4]
        request.kin_name = "fk"
        response = self.client.call(request)
        if isinstance(response, kinemaricsResponse):
            self.CurEndPos[0] = response.x
            self.CurEndPos[1] = response.y
            self.CurEndPos[2] = response.z
            self.CurEndPos[3] = response.Roll
            self.CurEndPos[4] = response.Pitch
            self.CurEndPos[5] = response.Yaw
            #print("CurEndPos: ",self.CurEndPos)
    
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
    
    
    def pixel_to_camera_depth(self,pixel_coords, depth):
        fx, fy, cx, cy = self.camera_info_K[0],self.camera_info_K[4],self.camera_info_K[2],self.camera_info_K[5]
        px, py = pixel_coords
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth
        return np.array([x, y, z])    

    
    def get_end_point_mat(self):
        #print("Get the current pose is ",self.CurEndPos)
        end_w,end_x,end_y,end_z = self.euler_to_quaternion(self.CurEndPos[3],self.CurEndPos[4],self.CurEndPos[5])
        endpoint_mat = self.xyz_quat_to_mat([self.CurEndPos[0],self.CurEndPos[1],self.CurEndPos[2]],[end_w,end_x,end_y,end_z])
        #print("endpoint_mat: ",endpoint_mat)
        return endpoint_mat
    
    

    def pub_arm(self, joints, id=6, angle=180, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        if len(joints) != 0: arm_joint.joints = joints
        else: arm_joint.joints = []
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
                #print("self.target_servox:",self.target_servox)
                
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
                #print("self.target_servoy = ",self.target_servoy)
                joint2 = 120 + self.target_servoy
                joint3 =  self.target_servoy / 4.5
                joint4 =  self.target_servoy / 3
                

        
        joints_0 = [self.target_servox/1, joint2, joint3, joint4, 90, 30]
        #print(joints_0)
        self.pub_arm(joints_0)
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
        request = kinemaricsRequest()
        request.tar_x = pose_T[0] 
        request.tar_y = pose_T[1] 
        request.tar_z = pose_T[2] +  (math.sqrt(request.tar_y**2+request.tar_x**2)-0.181)*0.2 #0.2为比例系数，根据实际夹取效果进行调整
        request.kin_name = "ik"
        request.Roll = self.CurEndPos[3]
        print("calcutelate_request: ",request)
        
        try:
            response = self.client.call(request)
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = response.joint2
            joints[2] = response.joint3
            if response.joint4>90:
                joints[3] = 90
            else:
                joints[3] = response.joint4
            joints[4] = 90 
            joints[5] = 20
            dist = math.sqrt(request.tar_y ** 2 + request.tar_x** 2)
            if dist>0.18 and dist<0.30:
                self.Buzzer()
                print("compute_joints: ",joints)
                self.pub_arm(joints)
                #time.sleep(3.5)
                self.move()
            else:
                print("It's too far to catch it!Please move it forward a bit. ")

        except Exception:
           rospy.loginfo("run error")
            
    def move(self):
        print("set_joint5: ",self.set_joint5)
        time.sleep(2.5)
        self.pub_arm([],5, 90, 2000)
        time.sleep(2.5)
        self.pub_arm([],6, 125, 2000)
        time.sleep(2.5)
        self.pub_arm([],2, 120, 2000)
        time.sleep(2.5)
        self.pub_arm(self.down_joint)
        time.sleep(2.5)
        self.pub_arm([],6, 90, 2000)
        time.sleep(2.5)
        self.pub_arm([],2, 90, 2000)
        time.sleep(2.5)
        self.pub_arm(self.init_joints)
        grab_status = Bool()
        grab_status.data = True
        self.pubGrab.publish(grab_status)
          
    def Buzzer(self):
        beep = Bool()
        beep.data = True
        self.pub_buzzer.publish(beep)
        time.sleep(1)
        beep.data = False
        self.pub_buzzer.publish(beep)
        time.sleep(1)


