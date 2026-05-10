#!/usr/bin/env python3
# encoding: utf-8
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool,String
from cv_bridge import CvBridge
import cv2 as cv
from Arm_Lib import Arm_Device
import getpass
import threading
encoding = ['16UC1', '32FC1']
import time
import math
import os
import yaml
from ament_index_python import get_package_share_directory
import transforms3d as tfs
import tf_transformations as tf         # ROS2使用tf_transformations

from sensor_msgs.msg import CompressedImage,Image
from std_msgs.msg import Int32, Bool,UInt16,Int16MultiArray
from dofbot_pro_KCF.astra_common import *
from dofbot_pro_interface.msg import *
from dofbot_pro_interface.srv import Kinemarics
import math

from arm_interface.msg import CurJoints

pkg_path = get_package_share_directory('dofbot_pro_driver')
offset_file = os.path.join(pkg_path,'config', 'offset_value.yaml')

with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))



class mono_Tracker(Node):
    def __init__(self):
        super().__init__('monoIdentify') 
        self.x_offset = offset_config.get('x_offset')
        self.y_offset = offset_config.get('y_offset')
        self.z_offset = offset_config.get('z_offset')
        self.point_pose = (0, 0, 0)
        self.circle = (0, 0, 0)
        self.hsv_range = ()
        self.circle_r = 0
        self.dyn_update = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
        self.cur_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 135.0]
        self.CurEndPos = [-0.000599999999999989, 0.11626166220790028, 0.09112890157533887, -1.0471975309176935,-0.0, 0.0]
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.90000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,4.90000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.end = 0
        self.cx = 0
        self.cy = 0
        self.Roi_init_src = []
        self.Roi_init_tar = []
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.Arm = Arm_Device()

        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", 10)
        self.pub_pos = self.create_publisher(Position, "/pos_xyz", 10)
        self.joint6_pub = self.create_publisher(Float32,'joint6',1)
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.subscription = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)

        self.depth_image_sub  = Subscriber(self, Image, "/camera/color/image_raw", qos_profile=1)
        self.rgb_image_sub = Subscriber(self, Image, "/camera/depth/image_raw", qos_profile=1)
        self.TimeSynchronizer = ApproximateTimeSynchronizer([self.depth_image_sub, self.rgb_image_sub],queue_size=10,slop=0.5)
        self.TimeSynchronizer.registerCallback(self.get_Src_Tar)
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',2)

        self.xy_subscription = self.create_subscription(Int16MultiArray,'corner_xy',self.GetXYCallback,qos_profile=1)

        self.down_joints_pub = self.create_publisher(CurJoints,'/down_joints',1)

        self.tracker_type = 'KCF'
        self.VideoSwitch = True
        self.img_flip = False
        self.grasp_joint = 140

        print("OpenCV Version: ",cv.__version__)
        self.gTracker = Tracker(tracker_type=self.tracker_type)
        self.Track_state = 'init'
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')		
        self.get_current_end_pos()
        #self.get_current_end_pos()
        self.pr_time = time.time()
        self.circle_r = 0 
        self.cur_distance = 0.0
        self.corner_x = self.corner_y = 0.0
        self.get_xy = False
        self.compute_roi = False
        self.done = True
        self.side = 0
        self.grasp_done = False
        self.start_move = False
        self.tar_x = 0.0
        self.tar_y = 0.0
        self.tar_z = 0.0
        
        
    def GetXYCallback(self,msg):
        print("msg: ",msg.data)
        self.Roi_init_src = (msg.data[0],msg.data[1],msg.data[2],msg.data[3])
        self.Roi_init_tar = (msg.data[4],msg.data[5],msg.data[6],msg.data[7])
        self.side = msg.data[8]
        self.Track_state = 'identify'
        self.get_xy = True
        time.sleep(2.5)

    
    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'


    def get_Src_Tar(self,color_frame,depth_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'bgr8')
        result_image = np.copy(rgb_image)
        result_image = cv.resize(result_image, (640, 480))
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        action = cv.waitKey(10) & 0xFF
        #result_image = cv.resize(result_image, (640, 480))


        
        if len(self.Roi_init_src)!=0 and len(self.Roi_init_tar)!=0:
            cv2.rectangle(result_image, (self.Roi_init_src[0], self.Roi_init_src[1]), (self.Roi_init_src[2], self.Roi_init_src[3]), (255,0,0), thickness=2)
            cv2.rectangle(result_image, (self.Roi_init_tar[0], self.Roi_init_tar[1]), (self.Roi_init_tar[2], self.Roi_init_tar[3]), (0,255,0), thickness=2)

            grasp_cx = (self.Roi_init_src[0] + self.Roi_init_src[2])/2
            grasp_cy = (self.Roi_init_src[1] + self.Roi_init_src[3])/2
            grasp_depth = depth_image_info[int(grasp_cy),int(grasp_cx)]/1000
           
            tar_cx = (self.Roi_init_tar[0] + self.Roi_init_tar[2])/2
            tar_cy = (self.Roi_init_tar[1] + self.Roi_init_tar[3])/2
            tar_depth = depth_image_info[int(tar_cy),int(tar_cx)]/1000    
            #print("tar_cx: ",tar_cx)
            #print("tar_cy: ",tar_cy)
            #print("tar_depth: ",tar_depth)
            if tar_depth!=0 and grasp_depth!=0:
                
                pose_grasp = self.compute_heigh(grasp_cx,grasp_cy,grasp_depth)
                pose_tar = self.compute_heigh(tar_cx,tar_cy,tar_depth)
                self.tar_x = pose_tar[0]
                self.tar_y = pose_tar[1]
                self.tar_z = pose_tar[2]
                #print("pose: ",pose)
                if self.done ==True:
                    self.get_current_end_pos()
                    self.done = False
                    self.grasp(pose_grasp[0],pose_grasp[1],pose_grasp[2])
                    #time.sleep(2.5)
                    #self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
                    #time.sleep(2.5)
                    print("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+")
                    #self.move()
                    #self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
                              
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time

        #cv2.rectangle(result_image, (130, 270), (200, 340), (255,0,0), thickness=2)
        cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)                              
        cv.imshow(self.windows_name, result_image)
        cv2.imshow("depth_image", depth_to_color_image)

    def compute_heigh(self,x,y,z):
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

    def get_current_end_pos(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')	
        request = Kinemarics.Request()
        request.cur_joint1 = self.cur_joints[0]
        request.cur_joint2 = self.cur_joints[1]
        request.cur_joint3 = self.cur_joints[2]
        request.cur_joint4 = self.cur_joints[3]
        request.cur_joint5 = self.cur_joints[4]
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

    def grasp(self,x,y,z):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')		
        self.get_current_end_pos()        
        request = Kinemarics.Request()        

        request.tar_y = y  + self.y_offset + 0.01
        request.tar_z = z  + self.z_offset + 0.01
        if x>0:
            request.tar_x = x +0.01 + self.x_offset
        else:
            request.tar_x = x - 0.01 + self.x_offset
        request.kin_name = "ik"
        request.roll = -0.785
        future = self.client.call_async(request)
        future.add_done_callback(self.grasp_get_ik_respone_callback)
   

    def move(self):
        request = Kinemarics.Request()
        px = self.tar_x + self.x_offset
        py = self.tar_y + self.y_offset 
        pz = self.tar_z + self.z_offset 
        #前
        if self.side == 1:
            py = py - 0.05
        #后
        elif self.side == 2:
            py = py + 0.05
        #左
        elif self.side == 3:
            px = px - 0.07
        #右
        elif self.side == 4:
            px = px + 0.07
        #上
        elif self.side == 5:
            pz = pz + 0.04
            
        request.tar_x =  px 
        request.tar_y = py 
        request.tar_z = pz 
        request.kin_name = "ik"
        request.roll = -1.0
        future = self.client.call_async(request)
        future.add_done_callback(self.move_get_ik_respone_callback)
        
    def grasp_get_ik_respone_callback(self, future):    
        try: 
            response = future.result()
            print("calcutelate_response: ",response)
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
            self.Arm.Arm_serial_servo_write6(joints[0],joints[1],joints[2],joints[3],joints[4],joints[5],2000)
            time.sleep(3.5)
            self.Arm.Arm_serial_servo_write(6, 135, 2000)
            time.sleep(2.5)
            self.init_joints[5] = self.grasp_joint
            self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)    
            time.sleep(2.5)
            self.move()
        except Exception:
           pass


    def move_get_ik_respone_callback(self, future):    
        try: 
            response = future.result()
            print("calcutelate_response: ",response)
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = response.joint2 
            joints[2] = response.joint3 
            if response.joint4>90:
                joints[3] = 90 
            else:
                joints[3] = response.joint4 
            joints[4] = 90
            joints[5] = self.grasp_joint
            self.Arm.Arm_serial_servo_write6(joints[0],joints[1],joints[2],joints[3],joints[4],joints[5],2000)
            time.sleep(2.5)
            self.Arm.Arm_serial_servo_write(6, 0, 2000)
            time.sleep(2.5)
            cur_joints = CurJoints()
            cur_joints.joints = [int(x) for x in joints]
            self.down_joints_pub.publish(cur_joints)
            self.init_joints[5] = 30
            self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
            time.sleep(2.5)
            self.largemodel_arm_done_pub.publish(String(data='change_pose_done'))
        except Exception:
           pass


    def GraspStatusCallback(self,msg):
         
        if msg.data == True:
            print("grasp is done.")
            self.largemodel_arm_done_pub.publish(String(data="grasp_obj_done"))
            self.Reset()


    def pub_arm(self, joints, id=6, angle=180.0, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        if len(joints) != 0: arm_joint.joints = joints
        else: arm_joint.joints = []
        self.pubPoint.publish(arm_joint)


def main(args=None):
    rclpy.init(args=args)
    kcf_tracker = mono_Tracker()
    try:
        rclpy.spin(kcf_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        kcf_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


