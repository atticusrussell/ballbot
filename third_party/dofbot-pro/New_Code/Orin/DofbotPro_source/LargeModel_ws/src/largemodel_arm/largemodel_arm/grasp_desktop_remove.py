#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import numpy as np
from std_msgs.msg import Float32,Bool,Int16
import time
import math
from arm_msgs.msg import ArmJoints
from arm_msgs.msg import ArmJoint
from arm_interface.srv import *
from arm_interface.msg import AprilTagInfo,CurJoints
from arm_interface.srv import ArmKinemarics
import transforms3d as tfs
import tf_transformations as tf
import threading
import yaml
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
offset_file = os.path.join(get_package_share_directory('arm_kin'), "param", "offset_value.yaml")
with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))
class GraspNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.grasp_flag = True

		self.init_joints =[90, 130, 0, 5, 90, 0]
		self.down_joint = [155, 35, 70, 5, 60,130]
		self.gripper_joint = 90
		self.CurEndPos =[0.16054173686606982, 0.00022737962578037993, 0.2159286461194661, 8.675891974613082e-05, 0.7853980110350363, 3.6519542644323194e-05]
		self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
		self.EndToCamMat = np.array([[ 0 ,0 ,1 ,-1.000e-01],
									 [-1  ,0 ,0  ,0],
									 [0  ,-1  ,0 ,4.99000000e-02],
									 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
									 
		self.sub_pos_info = self.create_subscription(AprilTagInfo,"PosInfo",self.pos_info_callback,1)
		self.sub_joint5 = self.create_subscription(Int16,"set_joint5",self.get_joint5_callback,1)
		self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.pub_SingleTargetAngle = self.create_publisher(ArmJoint, "arm_joint", 10)
		self.sub_cur_joints = self.create_subscription(CurJoints,"Curjoints",self.get_cur_joint_callback,1)
		self.pubGraspStatus = self.create_publisher(Bool,"grasp_done",1)
		self.client = self.create_client(ArmKinemarics, 'get_kinemarics')
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')			 
		print('init done')
		self.cur_joints = self.init_joints
		self.get_current_end_pos()
		self.joint6 = 140
		self.cur_tagId = 0
		self.x_offset = offset_config.get('x_offset')
		self.y_offset = offset_config.get('y_offset')
		self.z_offset = offset_config.get('z_offset')
		self.angle_radians = self.CurEndPos[4]

	def get_joint5_callback(self,msg):
		self.gripper_joint = msg.data
		print("get joint5 = ",msg.data)
		 
    
	def compute_pitch(self,pose_T):
		P1 = np.array([self.CurEndPos[0], self.CurEndPos[1], self.CurEndPos[2]])  # 点 P1 的坐标
		P2 = np.array([pose_T[0], pose_T[1],pose_T[2]])  # 点 P2 的坐标
		# 计算两点之间的欧几里得距离
		distance_xyz = np.linalg.norm(P2 - P1) 
		print("distance_xyz: ",distance_xyz)

		P3 = np.array([self.CurEndPos[0], self.CurEndPos[1]])  # 点 P1 的坐标
		P0 = np.array([0.0, 0.0])  # 点 P1 的坐标
		P4 = np.array([pose_T[0], pose_T[1]])  # 点 P2 的坐标
		# 计算两点之间的欧几里得距离
		distance_xy = np.linalg.norm(P4 - P0) 
		print("distance_xy: ",distance_xy)   

		#self.angle_radians =  np.arccos(distance_xy/distance_xyz)
		print("angle_radians: ",self.angle_radians)
    
	def get_cur_joint_callback(self,msg):
		print(msg.joints)
		self.cur_joints = msg.joints
		self.get_current_end_pos()

	def pos_info_callback(self,msg):
		print(msg)
		pos_x = msg.x
		pos_y = msg.y
		pos_z = msg.z
		self.cur_tagId = msg.id
		if pos_z!=0.0:
			#print("xyz id : ",pos_x,pos_y,pos_z,self.cur_tagId)
			
			camera_location = self.pixel_to_camera_depth((pos_x,pos_y),pos_z)
            #print("camera_location: ",camera_location)
			PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
            #PoseEndMat = np.matmul(self.xyz_euler_to_mat(camera_location, (0, 0, 0)),self.EndToCamMat)
			EndPointMat = self.get_end_point_mat()
			WorldPose = np.matmul(EndPointMat, PoseEndMat) 
            #WorldPose = np.matmul(PoseEndMat,EndPointMat)
			pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
			print("pose_T: ",pose_T)
			pose_T[0] = pose_T[0] + self.x_offset
			pose_T[1] = pose_T[1] + self.y_offset
			pose_T[2] = pose_T[2] + self.z_offset
			print("pose_T_add_offset: ",pose_T)
			print("pose_R: ",pose_R)
			self.compute_pitch(pose_T)
			if self.grasp_flag == True :
				print("Take it now.")
				self.grasp_flag = False
				grasp = threading.Thread(target=self.grasp, args=(pose_T,))
				grasp.start()
				grasp.join()
				
 
	def grasp(self,pose_T):
		print("------------------------------------------------")
		#print("pose_T: ",pose_T)
		request = ArmKinemarics.Request()
		request.tar_y = pose_T[1] 
		request.tar_z = pose_T[2] 
		request.tar_x = pose_T[0] 
		#print("request.tar_z: ",request.tar_z)
		request.kin_name = "ik"
		request.roll = 0.0 
		request.pitch  =  1.3962614348313127#夹取效果不佳可调整这个参数 1.04弧度等于60度表示末端目标姿态与桌面的夹角在60度 1.3962614348313127
		request.yaw = 0.0
		print("calcutelate_request: ",request)
		future = self.client.call_async(request)
		future.add_done_callback(self.get_ik_respone_callback)
           
	def get_ik_respone_callback(self, future):
		try:
			response = future.result()
			print("response: ",response)
			joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
			joints[0] = 180 - int(response.joint1) #response.joint1
			joints[1] = int(response.joint2)
			joints[2] = int(response.joint3)
			if response.joint4>90:
				joints[3] = 90
			else:
				joints[3] = int(response.joint4)
  
			if self.gripper_joint<0:
				joints[4] = abs(self.gripper_joint)
				if abs(self.gripper_joint)<90:
					joints[4] =180-joints[0]+joints[4] -90
                    
				else:
					joints[4]= joints[4] - joints[0] + 90
				if joints[4] >135:
					joints[4] =  joints[4] -90
				elif joints[4]<45:
					joints[4] =  joints[4] + 90
        

			if self.gripper_joint>0:
				joints[4] = 180 - abs(self.gripper_joint)
				if self.gripper_joint<90:
					joints[4] = joints[4]   - joints[0]
				else :
					joints[4] = joints[4] - (joints[0] - 90)
				if joints[4] >135:
					joints[4] =  joints[4] -90
				elif joints[4]<45:
					joints[4] =  joints[4] + 90
        


			joints[5] = 30
			print("compute_joints: ",joints)


			'''for i in range(6):
				if joints[i]<0:
					joints[i] = 0'''
			'''self.pubSingleArm(1, joints[0], 2000)
			time.sleep(2.0)
			self.pubSingleArm(5, joints[4], 2000)
			time.sleep(2.0)'''
			self.pubSixArm(joints)
			time.sleep(3.5)
			self.move()
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')

	def move(self):
		#print("self.gripper_joint = ",self.gripper_joint)
		#self.pubSingleArm(5, self.gripper_joint, 2000)
		#time.sleep(2.5)
		self.pubSingleArm(6, self.joint6, 2000)#机械臂夹紧（夹取目标）
		time.sleep(2.5)
		self.pubSingleArm(2, 120, 2000)#机械臂抬起
		self.pubSixArm([90, 180, 0, 0, 90,self.joint6])		
		# time.sleep(2.5)
		# if self.cur_tagId == 1:
		# 	self.down_joint = [155, 35, 70, 5, 60,self.joint6]
		# elif self.cur_tagId == 2:
		# 	self.down_joint = [180, 35, 60, 0, 90,self.joint6]
		# elif self.cur_tagId == 3:
		# 	self.down_joint = [28, 30, 70, 2, 90,self.joint6]
		# elif self.cur_tagId == 4:
		# 	self.down_joint = [0, 43, 48, 6, 90,self.joint6]
		if self.cur_tagId == 1:
			self.down_joint = [190, 5, 70, 15, 60,self.joint6]
		elif self.cur_tagId == 2:
			self.down_joint = [190, 5, 70, 15, 60,self.joint6]
		elif self.cur_tagId == 3:
			self.down_joint = [190, 5, 70, 15, 60,self.joint6]
		elif self.cur_tagId == 4:
			self.down_joint = [190, 5, 70, 15, 60,self.joint6]
		self.pubSixArm(self.down_joint)
		time.sleep(2.5)
		self.pubSingleArm(6, 30, 2000)#夹抓张开
		time.sleep(2.5)
		self.pubSingleArm(2, 90, 2000)#机械臂抬起
		time.sleep(2.5)
		print("self.cur_joints: ",self.cur_joints)
		self.pubSixArm(self.cur_joints)#机械臂复位
		time.sleep(2.5)
		self.grasp_flag = True
		grasp_done = Bool()
		grasp_done.data = True
		self.pubGraspStatus.publish(grasp_done)

  
	def pubSixArm(self, joints, id=6, angle=180.0, runtime=2000):
		arm_joints =ArmJoints()
		arm_joints.joint1 = int(joints[0])
		arm_joints.joint2 = int(joints[1])
		arm_joints.joint3 = int(joints[2])
		arm_joints.joint4 = int(joints[3])
		arm_joints.joint5 = int(joints[4])
		arm_joints.joint6 = int(joints[5])
		arm_joints.time = runtime
		self.pub_SixTargetAngle.publish(arm_joints)

	def pubSingleArm(self, joint_id,joint_angle,run_time):
		arm_joint =ArmJoint()
		arm_joint.joint = joint_angle
		arm_joint.id = joint_id
		arm_joint.time = run_time
		self.pub_SingleTargetAngle.publish(arm_joint)
		
	def get_current_end_pos(self):
		request = ArmKinemarics.Request()
		request.cur_joint1 = float(self.cur_joints[0])
		request.cur_joint2 = float(self.cur_joints[1])
		request.cur_joint3 = float(self.cur_joints[2])
		request.cur_joint4 = float(self.cur_joints[3])
		request.cur_joint5 = float(self.cur_joints[4])
		request.kin_name = "fk"
		future = self.client.call_async(request)
		future.add_done_callback(self.get_fk_respone_callback)

	def get_fk_respone_callback(self, future):
		try:
			response = future.result()
			#self.get_logger().info(f'Response received: {response.x}')
			self.CurEndPos[0] = response.x 
			self.CurEndPos[1] = response.y
			self.CurEndPos[2] = response.z 
			self.CurEndPos[3] = response.roll
			self.CurEndPos[4] = response.pitch
			self.CurEndPos[5] = response.yaw
			#self.get_logger().info(f'Response received: {self.CurEndPos}')
			print("self.CurEndPose: ",self.CurEndPos)
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')
		
	def get_end_point_mat(self):
        #print("Get the current pose is ",self.CurEndPos)
		end_w,end_x,end_y,end_z = self.euler_to_quaternion(self.CurEndPos[3],self.CurEndPos[4],self.CurEndPos[5])
		endpoint_mat = self.xyz_quat_to_mat([self.CurEndPos[0],self.CurEndPos[1],self.CurEndPos[2]],[end_w,end_x,end_y,end_z])
        #print("endpoint_mat: ",endpoint_mat)
		return endpoint_mat
        
	def pixel_to_camera_depth(self,pixel_coords, depth):
		fx, fy, cx, cy = self.camera_info_K[0],self.camera_info_K[4],self.camera_info_K[2],self.camera_info_K[5]
		px, py = pixel_coords
		x = (px - cx) * depth / fx
		y = (py - cy) * depth / fy
		z = depth
		return np.array([x, y, z])
        
	def xyz_euler_to_mat(self,xyz, euler, degrees=False):
		if degrees:
			mat = tfs.euler.euler2mat(math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]))
		else:
			mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
		mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
		return mat 
        
	def euler_to_quaternion(self,roll,pitch, yaw):
		quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
		qw = quaternion[3]
		qx = quaternion[0]
		qy = quaternion[1]
		qz = quaternion[2]
        #print("quaternion: ",quaternion )
		return np.array([qw, qx, qy, qz])
        
	def xyz_quat_to_mat(self,xyz, quat):
		mat = tfs.quaternions.quat2mat(np.asarray(quat))
		mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
		return mat
        
	def mat_to_xyz_euler(self,mat, degrees=False):
		t, r, _, _ = tfs.affines.decompose(mat)
		if degrees:
			euler = np.degrees(tfs.euler.mat2euler(r))
		else:
			euler = tfs.euler.mat2euler(r)
		return t, euler
		
def main():
	print('----------------------')
	rclpy.init()
	grasp = GraspNode('Grasp_Node')
	rclpy.spin(grasp)
