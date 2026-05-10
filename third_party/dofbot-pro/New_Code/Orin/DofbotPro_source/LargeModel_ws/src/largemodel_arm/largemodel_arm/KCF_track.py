import cv2
import os
import numpy as np
from cv_bridge import CvBridge
import cv2 as cv
from M3Pro_demo.Robot_Move import *
from arm_interface.srv import ArmKinemarics
from arm_interface.msg import AprilTagInfo,Position,CurJoints
from arm_msgs.msg import ArmJoints
from std_msgs.msg import Float32,Bool,UInt16
import time
from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import yaml
from M3Pro_demo.PID import *
import transforms3d as tfs
import tf_transformations as tf
import math
encoding = ['16UC1', '32FC1']
offset_file = "/home/jetson/yahboomcar_ws/src/arm_kin/param/offset_value.yaml"
with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))

print('init done')
class KCFTrackNode(Node):
	def __init__(self, name):
		super().__init__(name)
		# self.init_joints = [90, 150, 12, 20, 90, 0]
		self.init_joints = [
			90,
			130,
			0,
			5,
			90,
			0,
		]
		self.cur_distance = 0.0
		self.track_flag = True
		self.grasp_Dist = 240                                                                      
		self.linearx_PID = (0.5, 0.0, 0.2)
		self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
		self.EndToCamMat = np.array([[ 0 ,0 ,1 ,-1.00e-01],
									 [-1  ,0 ,0  ,0],
									 [0  ,-1  ,0 ,4.82000000e-02],
									 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
		# self.CurEndPos = [0.15326204031848129, 0.00021945213146852945, 0.3441362743447702, 6.061238269342215e-05, -0.03490672896787105, -2.909509128997985e-05]
		self.CurEndPos =  [0.16054173686606982, 0.00022737962578037993, 0.2159286461194661, 8.675891974613082e-05, 0.7853980110350363, 3.6519542644323194e-05]

		self.x_offset = offset_config.get('x_offset')
		self.y_offset = offset_config.get('y_offset')
		self.z_offset = offset_config.get('z_offset')
		self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)
		self.pubSixArm(self.init_joints)
		self.pos_sub = Subscriber(self, Position, '/pos_xyz')
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
		self.pub_CurJoints = self.create_publisher(CurJoints,"Curjoints",1)
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.reset_pub = self.create_publisher(Bool,"reset_flag",1)
		self.CmdVel_pub = self.create_publisher(Twist,"cmd_vel",1)
		self.pub_beep = self.create_publisher(UInt16, "beep", 10)
		self.cur_joints = self.init_joints
		self.pubSixArm(self.init_joints) 
		self.pub_cur_joints = self.create_publisher(CurJoints,"Curjoints",1)
		self.client = self.create_client(ArmKinemarics, 'get_kinemarics')
		self.get_current_end_pos()     
		self.ts = ApproximateTimeSynchronizer([self.pos_sub], 1, 0.5,allow_headerless=True)
		self.ts.registerCallback(self.callback)		
		self.adjust_dist = False
		self.start_grasp = False
		self.px = 0.0
		self.py = 0.0
		self.target_servox=90
		self.target_servoy=180
		self.xservo_pid = PositionalPID(0.25, 0.1, 0.05)
		self.yservo_pid = PositionalPID(0.25, 0.1, 0.05)
		self.y_out_range = False
		self.x_out_range = False
		self.a = 0
		self.b = 0
		self.XY_Track_flag = True
		self.Done_flag = True
		self.PID_init()

	def PID_init(self):
		self.linearx_pid = simplePID(self.linearx_PID[0] / 1000.0, self.linearx_PID[1] / 1000.0, self.linearx_PID[2] / 1000.0)

	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			self.pubSixArm(self.init_joints)
			self.XY_Track_flag = True
			self.Done_flag = True

	def move_dist(self,dist):
		linear_x = self.linearx_pid.compute(dist, self.grasp_Dist)
		self.pubVel(linear_x,0,0)


	def pubVel(self,vx,vy,vz):
		vel = Twist()
		vel.linear.x = float(vx)
		vel.linear.y = float(vy)
		vel.angular.z = float(vz)
		self.CmdVel_pub.publish(vel)

	def pubSixArm(self, joints, id=6, angle=180.0, runtime=2000):
		arm_joints =ArmJoints()
		arm_joints.joint1 = 180 - joints[0]
		arm_joints.joint2 = joints[1]
		arm_joints.joint3 = joints[2]
		arm_joints.joint4 = joints[3]
		arm_joints.joint5 = joints[4]
		arm_joints.joint6 = joints[5]
		arm_joints.time = runtime
		self.pub_SixTargetAngle.publish(arm_joints)

            
	def callback(self,msg):
		center_x, center_y = msg.x,msg.y
		cur_depth = msg.z
		# print("cur_depth: ",cur_depth)
		
		if (abs(center_x-320) >10 or abs(center_y-240)>10) and self.XY_Track_flag==True:
			self.XY_track(center_x,center_y)

		if abs(center_x-320) <10 and abs(center_y-240)<10 and self.Done_flag==True:
			self.adjust_dist = True		       
                
		if cur_depth==999.0:
			print("Invalid depth.")
			self.pubVel(0,0,0)
        
	def Beep_Loop(self):
		beep =UInt16()
		beep.data = 1
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = 0
		self.pub_beep.publish(beep)   


	def compute_heigh(self,x,y,z):
		camera_location = self.pixel_to_camera_depth((x,y),z)
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
		#print("pose_R: ",pose_R)
		return pose_T

	def get_end_point_mat(self):
        #print("Get the current pose is ",self.CurEndPos)
		self.get_current_end_pos()
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
			print("self.CurEndPose: ",self.CurEndPos)
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')

	def pubCurrentJoints(self):
		cur_joints = CurJoints()
		cur_joints.joints = self.cur_joints
		self.pub_cur_joints.publish(cur_joints) 

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
				self.target_servoy = int((target_valuey - 500) / 10) - 100#int((target_valuey - 500) / 10) - 55
				if self.target_servoy > 180: self.target_servoy = 180 #if self.target_servoy > 390: self.target_servoy = 390
				if self.target_servoy < 0: self.target_servoy = 0 
				joint2 = 120 + self.target_servoy
				joint3 =  self.target_servoy / 4.5
				joint4 =  self.target_servoy / 3
      
		joints_0 = [int(self.target_servox), int(joint2), int(joint3), int(joint4), 90, 30]
		print(joints_0)
		self.pubSixArm(joints_0)
		self.cur_joints = joints_0

		   
def main():

	rclpy.init()
	kcf_track = KCFTrackNode('KCFTrack_node')
	rclpy.spin(kcf_track)

