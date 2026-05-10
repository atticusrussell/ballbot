#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,Int16,UInt16,String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image
#common lib
import os
import threading
import math
from M3Pro_demo.follow_common import *
RAD2DEG = 180 / math.pi
print ("import finish")
cv_edition = cv.__version__
print("cv_edition: ",cv_edition)

import cv2
from arm_msgs.msg import ArmJoints
from message_filters import Subscriber, TimeSynchronizer,ApproximateTimeSynchronizer
from cv_bridge import CvBridge
encoding = ['16UC1', '32FC1']
from dt_apriltags import Detector
from M3Pro_demo.vutils import draw_tags
import numpy as np
from arm_interface.msg import AprilTagInfo,CurJoints
from M3Pro_demo.compute_joint5 import *

class LineDetect(Node):
	def __init__(self,name):
		super().__init__(name)


		#create a publisher
		self.pub_cmdVel = self.create_publisher(Twist,"/cmd_vel",1)
		self.pub_rgb = self.create_publisher(Image,"/linefollow/rgb",1)
		self.pub_Buzzer = self.create_publisher(UInt16,'/beep',1)
		#create a subscriber
		self.sub_JoyState = self.create_subscription(Bool,"/JoyState",self.JoyStateCallback,1)
		self.sub_laser = self.create_subscription(LaserScan,"/scan1",self.registerScan,1)
		self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)

		self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
		self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
		self.pub_SixTargetAngle = self.create_publisher(ArmJoints, "arm6_joints", 10)		
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)
		self.TargetJoint6_pub = self.create_publisher(Int16, "set_joint6", 10)
		self.pub_cur_joints = self.create_publisher(CurJoints,"Curjoints",1)
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
		self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',1)	
		self.init_joints = [90, 90, 12, 20, 90, 0]
		while not self.pub_SixTargetAngle.get_subscription_count():
			self.pubSixArm(self.init_joints)
			time.sleep(0.1)
		self.pubSixArm(self.init_joints)
		self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], 1, 0.5)
		self.ts.registerCallback(self.callback)

		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()

		self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
		self.move_flag = True 
		self.pubPos_flag = True

		self.declare_param()
		self.Joy_active = False
		self.img = None
		self.circle = ()
		self.hsv_range = ()
		self.Roi_init = ()
		self.warning = 1
		self.Start_state = True
		self.dyn_update = False
		self.Buzzer_state = False
		self.select_flags = False
		self.Track_state = 'identify'
		self.windows_name = 'frame'
		self.cols, self.rows = 0, 0
		self.Mouse_XY = (0, 0)
		self.hsv_text = "/home/jetson/yahboomcar_ws/src/M3Pro_demo/M3Pro_demo/LineFollowHSV.text"
		self.color = color_follow()
		self.scale = 1000
		self.FollowLinePID = (50, 0, 10)
		self.RemovePID = (40, 0, 15.0)
		self.linear = 0.2
		#self.LaserAngle = 60
		self.PID_init()	
		self.img_flip = False
		self.refresh  = False	

		self.pubCurrentJoints()
		self.tags = []
		self.depth_image_info = []
		self.joint5 = Int16()
		self.joint6 = Int16()
		self.joint6.data = 120
		self.Start_ = True
		self.start_time = time.time()
		self.count = True
		#self.ResponseDist = 0.25
		self.front_warning = 0
		self.Joy_active = False
		print("Init Done.")
		print("----------------------------")
		print("self.LaserAngle: ",self.LaserAngle)
		print("self.ResponseDist: ",self.ResponseDist)



	def JoyStateCallback(self, msg):
		if not isinstance(msg, Bool): return
		self.Joy_active = msg.data

	def registerScan(self, scan_data):
		self.front_warning = 0
		if not isinstance(scan_data, LaserScan): return
		ranges = np.array(scan_data.ranges)
		for i in range(len(ranges)):
			angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
			if (abs(angle) < self.LaserAngle*0.5 or abs(angle) > 360 - self.LaserAngle*0.5) and ranges[i] !=0.0 :
				if ranges[i] <= self.ResponseDist * 1.5: 
					#print("-+-+-+-+-+-+-+-+-")
					self.front_warning += 1			

	
	def pubCurrentJoints(self):
		cur_joints = CurJoints()
		cur_joints.joints = self.init_joints
		self.pub_cur_joints.publish(cur_joints)

    
	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			self.move_flag = True
			self.pubPos_flag  = True
			if len(self.tags) == 0:
				self.Track_state = 'tracking'

	def callback(self,color_frame,depth_frame):
        # 将画面转为 opencv 格式
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
		rgb_image = np.copy(rgb_image)

		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
		#depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1.0), cv2.COLORMAP_JET)
		depth_img = cv2.resize(depth_image, (640, 480))
		self.depth_image_info = depth_img.astype(np.float32)
        
		self.tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
		self.tags = sorted(self.tags, key=lambda tag: tag.tag_id) 
		draw_tags(rgb_image, self.tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
        
		#depth_image
		depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
		#depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1.0), cv2.COLORMAP_JET)
		frame = cv2.resize(depth_image, (640, 480))
		depth_image_info = frame.astype(np.float32)
		action = cv2.waitKey(1)
		if self.count==True and self.Start_==True:
			if (time.time() - self.start_time)>3:
				self.Track_state = 'tracking'
				self.count = False
		result_img,bin_img = self.process(rgb_image,action)
		result_img = cv2.cvtColor(result_img, cv2.COLOR_RGB2BGR)
		if len(bin_img) != 0: cv.imshow('frame', ManyImgs(1, ([result_img, bin_img])))
		else:cv.imshow('frame', result_img)

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

	def declare_param(self):
		#HSV
		self.declare_parameter("Hmin",0)
		self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
		self.declare_parameter("Smin",85)
		self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
		self.declare_parameter("Vmin",126)
		self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
		self.declare_parameter("Hmax",9)
		self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
		self.declare_parameter("Smax",253)
		self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
		self.declare_parameter("Vmax",253)
		self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
        #PID
		self.declare_parameter("Kp",60)
		self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
		self.declare_parameter("Ki",0)
		self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
		self.declare_parameter("Kd",20)
		self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
		#other
		self.declare_parameter("scale",1000)
		self.scale = self.get_parameter('scale').get_parameter_value().integer_value
		self.declare_parameter("LaserAngle",60)
		self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().integer_value
		self.declare_parameter("linear",0.18)
		self.linear = self.get_parameter('linear').get_parameter_value().double_value
		self.declare_parameter("ResponseDist",0.25)
		self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
		self.declare_parameter('refresh',False)
		self.refresh = self.get_parameter('refresh').get_parameter_value().bool_value

	def PID_init(self):
		self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0])  
		self.Remove_PID_controller = simplePID(
            [0, 0],
            [self.RemovePID[0] / 1.0 / (self.scale), self.RemovePID[0] / 1.0 / (self.scale)],
            [self.RemovePID[1] / 1.0 / (self.scale), self.RemovePID[1] / 1.0 / (self.scale)],
            [self.RemovePID[2] / 1.0 / (self.scale), self.RemovePID[2] / 1.0 / (self.scale)]) 
		
	def onMouse(self, event, x, y, flags, param):
		if event == 1:
			self.Track_state = 'init'
			self.select_flags = True
			self.Mouse_XY = (x,y)
		if event == 4:
			self.select_flags = False
			self.Track_state = 'mouse'
		if self.select_flags == True:
			self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
			self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
			self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

	def execute(self, point_x, color_radius):

		if self.Joy_active == True:
			if self.Start_state == True:
				self.PID_init()
				self.Start_state = False
			return
		self.Start_state = True
		if color_radius == 0: 
			print("Not Found")
			self.largemodel_arm_done_pub.publish( String(data='follw_line_clear_future_done'))

			self.pub_cmdVel.publish(Twist())
		else:
			twist = Twist()
			b = UInt16()
			[z_Pid, _] = self.PID_controller.update([(point_x - 320)*1.0/16, 0])
			#[z_Pid, _] = self.PID_controller.update([(point_x - 10)*1.0/16, 0])
			if self.img_flip == True: twist.angular.z = -z_Pid #-z_Pid
			#else: twist.angular.z = (twist.angular.z+z_Pid)*0.2
			else: twist.angular.z = +z_Pid
			#point_x = point_x
			#twist.angular.z=-(point_x-320)*1.0/128.0

			twist.linear.x = 0.1
			if self.front_warning > 10:
				print("Obstacles ahead !!!")
				self.pub_cmdVel.publish(Twist())
				self.Buzzer_state = True
				b.data = 1
				self.pub_Buzzer.publish(b)
			else:
				if self.Buzzer_state == True:
					b.data = 0
					for i in range(3): self.pub_Buzzer.publish(b)
					self.Buzzer_state = False
                
				if abs(point_x-320)<40:
                #if abs(point_x-30)>40:
					twist.angular.z=0.0
				if self.Joy_active == False:
					self.pub_cmdVel.publish(twist)
				else:
					twist.angular.z=0.0
                

	def process(self, rgb_img, action):
		#print("********************************")
		binary = []
		rgb_img = cv.resize(rgb_img, (640, 480))
		# self.Track_state = 'tracking'
		if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
		if action == 32: self.Track_state = 'tracking'

		elif action == ord('i') or action == 105: self.Track_state = "identify"
		elif action == ord('r') or action == 114: self.Reset()
		elif action == ord('q') or action == 113: self.cancel()
		if self.Track_state == 'init':
			cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
			cv.setMouseCallback(self.windows_name, self.onMouse, 0)
			if self.select_flags == True:
				cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
				cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
				if self.Roi_init[0]!=self.Roi_init[2] and self.Roi_init[1]!=self.Roi_init[3]:
					rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
					self.dyn_update = True
				      
				    
			else: 
					self.Track_state = 'init'
		elif self.Track_state == "identify":
			#print(self.circle[0])
			if os.path.exists(self.hsv_text): self.hsv_range = read_HSV(self.hsv_text)
			else: self.Track_state = 'init'
		if self.Track_state != 'init' and len(self.hsv_range) != 0:
			rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
			if self.dyn_update == True:
				write_HSV(self.hsv_text, self.hsv_range)
				self.Hmin  = rclpy.parameter.Parameter('Hmin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][0])
				self.Smin  = rclpy.parameter.Parameter('Smin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][1])
				self.Vmin  = rclpy.parameter.Parameter('Vmin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][2])
				self.Hmax  = rclpy.parameter.Parameter('Hmax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][0])
				self.Smax  = rclpy.parameter.Parameter('Smax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][1])
				self.Vmax  = rclpy.parameter.Parameter('Vmax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][2])

				
				all_new_parameters = [self.Hmin,self.Smin,self.Vmin,self.Hmax,self.Smax,self.Vmax]
				self.set_parameters(all_new_parameters)
				self.dyn_update = False
		if self.Track_state == 'tracking' :
			if len(self.circle) != 0:
				threading.Thread(target=self.execute, args=(self.circle[0], self.circle[2])).start()
		else:
			if self.Start_state == True:
				#self.pub_cmdVel.publish(Twist())
				self.Start_state = False
		if len(self.tags)>0 and self.Track_state!="Remove":
			self.Track_state = "identify"
			self.pub_cmdVel.publish(Twist())
			print("Find the apriltag.")
			self.Track_state = "Remove"
		if  self.Track_state == "Remove":
			print("len(tags) = ",len(self.tags))
			if len(self.tags)>0 :
				center_x, center_y = self.tags[0].center
				if (abs(center_x-320) >10 or abs(center_y-400)>10) and self.move_flag == True: 
					print("adjusting.")
					self.remove_obstacle(center_x, center_y)
				if abs(center_x-320) <10 and abs(center_y-400)<10:
					self.pubVel(0.0, 0.0)
					print("start crawling.")
					time.sleep(3)
					c_dist = self.depth_image_info[int(center_y),int(center_x)]/1000
					if c_dist!=0 and self.pubPos_flag == True:
						self.move_flag = False
						self.pubPos_flag = False
						pos = AprilTagInfo()
						pos.id = self.tags[0].tag_id
						pos.x = center_x
						pos.y = center_y
						pos.z = c_dist
						vx = int(self.tags[0].corners[0][0]) - int(self.tags[0].corners[1][0])
						vy = int(self.tags[0].corners[0][1]) - int(self.tags[0].corners[1][1])
						target_joint5 = compute_joint5(vx,vy)
						print("target_joint5: ",target_joint5)
						self.joint5.data = int(target_joint5)
						print("tag_id: ",self.tags[0].tag_id)
						print("center_x, center_y: ",center_x, center_y)
						print("depth: ",c_dist)
						self.pos_info_pub.publish(pos)
						self.TargetJoint5_pub.publish(self.joint5)
						self.TargetJoint5_pub.publish(self.joint5)
					else:
						print("Invalid distance.")
                
		return rgb_img, binary

	def pubVel(self,vx,vy):
		vel = Twist()
		vel.linear.x = vx
		vel.linear.y = vy

		self.pub_cmdVel.publish(vel)
        
	def remove_obstacle(self,point_x, point_y):
		[y, x] = self.Remove_PID_controller.update([(point_x - 320) / 10.0, (point_y - 400) / 10.0])
		if x >= 0.10: x = 0.10 #0.10
		elif x <= -0.10: x = -0.10
		if y >= 0.10: y = 0.10
		elif y <= -0.10: y = -0.10
		self.pubVel(x, y)        
		
	def JoyStateCallback(self,msg):
		if not isinstance(msg, Bool): return
		self.Joy_active = msg.data
		#self.pub_cmdVel.publish(Twist())


	def Reset(self):
		self.PID_init()
		self.Track_state = 'init'
		self.hsv_range = ()
		self.Joy_active =False
		self.Mouse_XY = (0, 0)
		self.pub_cmdVel.publish(Twist())
		print("Reset succes!!!")

	def get_param(self):
		#hsv
		self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
		self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
		self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
		self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
		self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
		self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
		#kpi
		self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
		self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
		self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
		self.FollowLinePID = (self.Kp,self.Ki,self.Kd)
		#
		self.scale = self.get_parameter('scale').get_parameter_value().integer_value
		self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().integer_value
		self.linear = self.get_parameter('linear').get_parameter_value().double_value
		self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
		self.refresh = self.get_parameter('refresh').get_parameter_value().bool_value

		
def main():
	rclpy.init()
	linedetect = LineDetect("follow_line")
	print("start it")
	try:
		rclpy.spin(linedetect)
	except KeyboardInterrupt:
		pass
	finally:
		linedetect.pub_cmdVel.publish(Twist())
		linedetect.destroy_node()
		rclpy.shutdown()
	

	
	
	
