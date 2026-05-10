import cv2
import os
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import message_filters

from cv_bridge import CvBridge
import cv2 as cv

from largemodel_arm.color_common import *
from dofbot_pro_interface.srv import Kinemarics
from arm_interface.msg import AprilTagInfo,CurJoints
from dofbot_pro_interface.msg import *
from std_msgs.msg import Float32,Bool,Int16,UInt16,String
encoding = ['16UC1', '32FC1']
import time
import transforms3d as tfs
import tf_transformations as tf
import yaml
import math
from rclpy.node import Node
import rclpy
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import threading
from ament_index_python import get_package_share_directory
from dofbot_pro_driver.compute_joint5 import *


pkg_path = get_package_share_directory('dofbot_pro_driver')
offset_file = os.path.join(pkg_path,'config', 'offset_value.yaml')

with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))
print('init done')
class ColorRecognizeNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()
		self.pub_pos_flag = True
		self.pr_time = time.time()
		self.cnt = 0
		self.cur_distance = 0.0
		self.track_flag = True
		self.prev_dist = 0
		self.prev_angular = 0
		self.prev_roll = 0
		self.minDist = 400
		self.xy_track_flag = True
		self.linearx_PID = (0.5, 0.0, 0.2)  
		self.lineary_PID = (0.2, 0.0, 0.1)
		self.angz_PID = (0.5, 0.0, 0.5)
		self.CurEndPos = [-0.006,0.116261662208,0.0911289015753,-1.04719,-0.0,0.0]
		self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
		self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.90000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,4.90000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])

		self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
		self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
		self.CmdVel_pub = self.create_publisher(Twist,"cmd_vel",1)
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')	
		self.pub_beep = self.create_publisher(UInt16, "beep", 10)
		self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)
		self.pub_reset_gesture = self.create_publisher(Bool,"reset_gesture",1)
		self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',1)

		self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", qos_profile=1)
		while not self.pubPoint.get_subscription_count():
			self.pub_arm(self.init_joints)
			time.sleep(0.1)         
		self.pub_arm(self.init_joints)
		#self.PID_init()
		self.get_current_end_pos()
		self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)

		self.tag_size = 0.05
		self.camera_matrix = None
		self.dist_coeffs = None
		self.rotation_direction = 1
		self.roll_track_flag = True
		self.start_grasp = False
		self.move_y = True

		self.x_direction = 1
		self.x_roll_track_flag = True
		self.y_roll_track_flag = False
		self.x_track_done = False
		self.roll_track_done = False
		self.y_track_done = False
		self.Target_Shape = "Square"
		self.x_offset = offset_config.get('x_offset')
		self.y_offset = offset_config.get('y_offset')
		self.z_offset = offset_config.get('z_offset')
		self.adjust_dist = True
		self.prev_dist = 0
		self.linearx_PID = (0.5, 0.0, 0.2)
		self.linearx_pid = simplePID(self.linearx_PID[0] / 1000.0, self.linearx_PID[1] / 1000.0, self.linearx_PID[2] / 1000.0)
		self.grasp_Dist = 200.0
		self.done_flag = False
		self.target_color = 0
		self.red_hsv_text = "/home/jetson/LargeModel_ws/src/largemodel_arm/largemodel_arm/red_colorHSV.text"
		self.green_hsv_text = "/home/jetson/LargeModel_ws/src/largemodel_arm/largemodel_arm/green_colorHSV.text"
		self.blue_hsv_text = "/home/jetson/LargeModel_ws/src/largemodel_arm/largemodel_arm/blue_colorHSV.text"
		self.yellow_hsv_text = "/home/jetson/LargeModel_ws/src/largemodel_arm/largemodel_arm/yellow_colorHSV.text"
		self.hsv_range = ()
		self.select_flags = False
		self.gTracker_state = False
		self.windows_name = 'frame'
		self.Track_state = 'init'
		self.Mouse_XY = (0, 0)
		self.cols, self.rows = 0, 0
		self.Roi_init = ()
		self.color = color_detect()
		self.cur_color = None
		self.text_color = (0,0,0)
		self.cx = 0
		self.cy = 0
		self.circle_r = 0
		self.valid_dist = True
		self.CX_list = []
		self.CY_list = []
		self.R_list = []
		self.detect_flag = False
		self.index = None
		self.joint5 = Int16()
		self.corners = np.empty((4, 2), dtype=np.int32)

		self.declare_parameter('target_color', 0.0)
		self.target_color = int(self.get_parameter('target_color').get_parameter_value().double_value)
		print("Get self.target_color is ",self.target_color)

		self.declare_parameter('target_high', 0.0)
		self.target_high = self.get_parameter('target_high').get_parameter_value().double_value * 1.0
		print("Get self.target_high is ",self.target_high)        

		self.statr_flag  = False
		self.count = True
		self.start_time = time.time()
		self.found_cnt = 0
		self.compute_height = False
        
		print("Init done.")

	def pub_arm(self, joints, id=6, angle=180.0, runtime=1500):
		arm_joint = ArmJoint()
		arm_joint.id = id
		arm_joint.angle = angle
		arm_joint.run_time = runtime
		if len(joints) != 0: arm_joint.joints = joints
		else: arm_joint.joints = []
		self.pubPoint.publish(arm_joint)
              
	def Beep_Loop(self):
		beep = UInt16()
		beep.data = 1
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = 0
		self.pub_beep.publish(beep)

	def get_current_end_pos(self):
		request = Kinemarics.Request()
		request.cur_joint1 = float(self.init_joints[0])
		request.cur_joint2 = float(self.init_joints[1])
		request.cur_joint3 = float(self.init_joints[2])
		request.cur_joint4 = float(self.init_joints[3])
		request.cur_joint5 = float(self.init_joints[4])
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


	def camera_info_callback(self, msg):
        # 获取相机内参矩阵和畸变系数
		self.camera_matrix = np.array(msg.k).reshape(3, 3)
		self.dist_coeffs = np.array(msg.d)

	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			self.pub_pos_flag = True
			self.done_flag = True
			self.adjust_dist = True
			self.detect_flag = False
			self.compute_height = True
			self.valid_dist = True
			#self.largemodel_arm_done_pub.publish(String(data="color_remove_higher_done"))
			time.sleep(3.0)
                     
    
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
                

	def Reset(self):
		self.hsv_range = ()
		self.circle = (0, 0, 0)
		self.Mouse_XY = (0, 0)
		self.Track_state = 'init'
		print("Change the state.")
		self.cx = 0
		self.cy = 0
		self.pubPos_flag = False
		self.target_color = 0


	def onMouse(self, event, x, y, flags, param):
		if event == 1:
			self.Track_state = 'init'
			self.select_flags = True
			self.Mouse_XY = (x, y)
		if event == 4:
			self.select_flags = False
			self.Track_state = 'mouse'
		if self.select_flags == True:
			self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
			self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
			self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
            
	def callback(self,color_frame,depth_frame):
        # 将画面转为 opencv 格式
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
		rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
		result_image = np.copy(rgb_image)
		#depth_image
		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
		frame = cv.resize(depth_image, (640, 480))
		depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1.0), cv2.COLORMAP_JET)
		depth_image_info = frame.astype(np.float32)  
		key = cv2.waitKey(10)& 0xFF
		result_frame, binary = self.process(rgb_image,key)
		

		if self.count==True:
			if (time.time() - self.start_time)>8:
				self.done_flag = True
				self.compute_height = True
				self.count = False
		if len(self.CX_list)>0 and self.done_flag==True and self.compute_height==True:
			for i in range(len(self.CX_list)):
				if self.R_list_[i]>30:
					cx = int(self.CX_list[i])
					cy = int(self.CY_list[i])
					dist = depth_image_info[int(cy),int(cx)]/1000
					#self.get_logger().info(f'dist:{dist}')
					pose = self.compute_heigh(cx,cy,dist)
					compute_heigh = round(pose[2],2)*100
					heigh = 'heigh: ' + str(compute_heigh) + ' cm'
					#self.get_logger().info(heigh)
					#print(heigh)
					cv.putText(result_frame, heigh, (int(cx)+10, int(cy)-25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
					if  compute_heigh > self.target_high and self.pub_pos_flag == True :
						self.index = i
						self.detect_flag = True
						self.compute_height = False
						self.found_cnt = self.found_cnt + 1
                        
			
					if self.detect_flag == True and self.index != None:
						cx = int(self.CX_list[self.index])
						cy = int(self.CY_list[self.index])
						vx = self.corners[0][0][0] - self.corners[1][0][0]
						vy = self.corners[0][0][1] - self.corners[1][0][1] 
						target_joint5 = compute_joint5(vx,vy)
						dist = depth_image_info[int(cy),int(cx)]/1000 
						cv.putText(result_frame, heigh, (int(cx)+10, int(cy)-25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
						if dist!=0 and self.pub_pos_flag == True:
							pos = AprilTagInfo()
							pos.id = self.target_color
							pos.x = float(cx)
							pos.y = float(cy)
							pos.z = float(dist)
							if self.pub_pos_flag == True:
								self.joint5.data = int(target_joint5)
								self.TargetJoint5_pub.publish(self.joint5) 
								self.index = None
								self.pub_pos_flag = False
								self.done_flag = False
								self.pos_info_pub.publish(pos)  
								print("Publish the position.")

			if self.detect_flag == False and self.found_cnt>0 and self.pub_pos_flag == True:
				self.found_cnt = 0
				self.largemodel_arm_done_pub.publish(String(data="color_remove_higher_done"))
				self.pub_pos_flag = False
                
			elif 	self.detect_flag == False  and self.pub_pos_flag == True:
				# self.Beep_Loop()
				print("Do not find target hight.")
				self.largemodel_arm_done_pub.publish(String(data="color_remove_higher_done"))
				self.pub_pos_flag = False

		if len(binary) != 0: 
			cv.imshow(self.windows_name, ManyImgs(1, ([result_frame, binary])))
		else:
			cv.imshow(self.windows_name, result_frame)
		# cv.imshow('1111', result_image)

	def process(self,rgb_img,key):
		rgb_img = cv.resize(rgb_img, (640, 480))
		binary = []

		
		if key == ord('c') or key == ord('C'):
			self.target_color = 0
			self.Reset()        
		elif key == ord('i') or key == ord('I') or self.target_color!=0: self.Track_state = "identify"
		#print("self.Track_state: ",self.Track_state)
		if self.Track_state == 'init':
			cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
			cv.setMouseCallback(self.windows_name, self.onMouse, 0)
			if self.select_flags == True:
				cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
				cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
				if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
					print("self.hsv_range: ",self.hsv_range)
					rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
					self.gTracker_state = True
					self.dyn_update = True
				else: self.Track_state = 'init'
                    
		elif self.Track_state == "identify":
			if self.target_color == 1:
				self.hsv_range = read_HSV(self.red_hsv_text)
				self.cur_color = "red"
				self.text_color = (0, 0, 255)

                
			elif self.target_color == 2:
				self.hsv_range = read_HSV(self.green_hsv_text)
				self.cur_color = "green"
				self.text_color = (0, 255, 0)

                
			elif self.target_color == 3:
				self.hsv_range = read_HSV(self.blue_hsv_text)
				self.cur_color = "blue"
				self.text_color = (255, 0, 0)
                
			elif self.target_color == 4:
				self.hsv_range = read_HSV(self.yellow_hsv_text)
				self.cur_color = "yellow"
				self.text_color = (255, 255, 0)
                
			else: 
				self.Track_state = 'init'

		if self.Track_state != 'init':
			if len(self.hsv_range) != 0:
				rgb_img, binary, self.CX_list,self.CY_list,self.R_list_,self.corners= self.color.object_follow_list(rgb_img, self.hsv_range)

		rgb_img = cv2.putText(rgb_img, self.cur_color, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, self.text_color, 2)             
		return rgb_img, binary



		   
def main():
	print('----------------------')
	rclpy.init()
	color_recognize = ColorRecognizeNode('ColorRecognize_node')
	rclpy.spin(color_recognize)

           