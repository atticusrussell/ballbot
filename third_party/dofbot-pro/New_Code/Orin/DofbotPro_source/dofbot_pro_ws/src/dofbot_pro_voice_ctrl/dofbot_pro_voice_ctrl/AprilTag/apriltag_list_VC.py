#!/usr/bin/env python
# -*- coding: utf-8 -*-
print("语音指令词如下：")
print("------移除高度异常机器码------")
import rclpy
from rclpy.node import Node
import cv2 
import numpy as np
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float32, Bool,Int8
from cv_bridge import CvBridge
import cv2 as cv
from dt_apriltags import Detector
import transforms3d as tfs
import tf_transformations as tf         # ROS2使用tf_transformations
import threading
from dofbot_pro_driver.vutils import draw_tags
from dofbot_pro_interface.srv import Kinemarics
from dofbot_pro_interface.msg import *
import pyzbar.pyzbar as pyzbar
from std_msgs.msg import Float32,Bool
import time
import queue
import math
from Arm_Lib import Arm_Device
import os
encoding = ['16UC1', '32FC1']
from ament_index_python import get_package_share_directory
import yaml
pkg_path = get_package_share_directory('dofbot_pro_driver')
offset_file = os.path.join(pkg_path,'config', 'offset_value.yaml')


with open(offset_file, 'r') as file:
    offset_config = yaml.safe_load(file)
print(offset_config)
print("----------------------------")
print("x_offset: ",offset_config.get('x_offset'))
print("y_offset: ",offset_config.get('y_offset'))
print("z_offset: ",offset_config.get('z_offset'))
exit_code = os.system('ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 "data: 40"')
class AprilTagDetectNode(Node):
    def __init__(self):
        super().__init__('apriltag_detect')
        self.x_offset = offset_config.get('x_offset')
        self.y_offset = offset_config.get('y_offset')
        self.z_offset = offset_config.get('z_offset')
        self.Arm = Arm_Device()
        self.start_flag = False
        self.no_found = True
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
        self.depth_image_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        self.rgb_image_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub],queue_size=10,slop=0.5)
        self.ts.registerCallback(self.TagDetect)
        
        self.sub_voice = self.create_subscription(Int8,"voice_result",self.getVoiceResultCallBack,1)
        self.pub_playID = self.create_publisher(Int8,"player_id", 1)
        self.subscription = self.create_subscription( Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)

        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.pubPoint = self.create_publisher(ArmJoint,"TargetAngle", 1)
        self.pub_playID = self.create_publisher(Int8,"player_id", 1)

        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')

        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.pubPos_flag = True
        self.pr_time = time.time()
        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        self.target_id = 31
        self.Center_x_list = []
        self.Center_y_list = []
        self.heigh = 0.0
        self.CurEndPos = [-0.006,0.116261662208,0.0911289015753,-1.04719,-0.0,0.0]
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.90000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,4.90000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        '''self.EndToCamMat = np.array([[0,0,-1,-0.247],
                                     [ -1.000, 0.000,0,-0.19],
                                     [0.000,1.00,0.000,0.386],
                                     [0.000,0.000, 0.000,1.000]])'''
        self.get_current_end_pos()
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
        
        
    def getVoiceResultCallBack(self,msg):
        if msg.data == 99:
            play_id = Int8()
            play_id.data = 45
            self.pub_playID.publish(play_id)
            self.start_flag = True
            print("Start removing.") 

    def get_current_end_pos(self):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'dofbot_kinematics' not available!")
            return
        request = Kinemarics.Request()
        request.cur_joint1 = self.init_joints[0]
        request.cur_joint2 = self.init_joints[1]
        request.cur_joint3 = self.init_joints[2]
        request.cur_joint4 = self.init_joints[3]
        request.cur_joint5 = self.init_joints[4]
        request.kin_name = "fk"
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0) 
        response = future.result()
        if isinstance(response, Kinemarics.Response):
            self.CurEndPos[0] = response.x
            self.CurEndPos[1] = response.y
            self.CurEndPos[2] = response.z
            self.CurEndPos[3] = response.roll
            self.CurEndPos[4] = response.pitch
            self.CurEndPos[5] = response.yaw   

    def TagDetect(self,color_frame,depth_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        result_image = np.copy(rgb_image)
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        depth_to_color_image = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=1.0), cv.COLORMAP_JET)
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
        tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
        draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
        self.Center_x_list = list(range(len(tags)))
        self.Center_y_list = list(range(len(tags)))
        key = cv2.waitKey(10)

        if len(tags) > 0 :
            for i in range(len(tags)):
                center_x, center_y = tags[i].center
                cv2.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
                
                self.Center_x_list[i] = center_x
                self.Center_y_list[i] = center_y
                cx = center_x
                cy = center_y
                cz = depth_image_info[int(cy),int(cx)]/1000
                pose = self.compute_heigh(cx,cy,cz)
                heigh = round(pose[2],4)*1000 + 10
                heigh = 'heigh: ' + str(heigh) + 'mm'
                dist_detect = math.sqrt(pose[1] ** 2 + pose[0]** 2)
                dist_detect = round(dist_detect,3)*1000
                dist = 'dist: ' + str(dist_detect) + 'mm'
                cv.putText(result_image, heigh, (int(cx)+5, int(cy)-15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv.putText(result_image, dist, (int(cx)+25, int(cy)-35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                print("Pose: ",pose)
                if pose[2]>0.04:
                    self.no_found = False
                    tag = AprilTagInfo()
                    tag.id = tags[i].tag_id
                    tag.x = self.Center_x_list[i]
                    tag.y = self.Center_y_list[i]
                    tag.z = depth_image_info[int(tag.y),int(tag.x)]/1000
                    if tag.z!=0 and self.start_flag == True:
                        # print(tags[i].tag_id)
                        self.pos_info_pub.publish(tag)
                        self.pubPos_flag = False
                    elif tag.z==0:
                        print("Invalid distance.")
            if self.no_found == True and self.start_flag == True:
                print("Not found!")
                self.no_found = False
                self.start_flag = False
                play_id = Int8()
                play_id.data = 1
                self.pub_playID.publish(play_id)
            
                           
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("result_image", result_image)
        cv2.imshow("depth_image", depth_to_color_image)
        key = cv2.waitKey(1)

    def GraspStatusCallback(self,msg):
        if msg.data == True:
            self.pubPos_flag = True
            time.sleep(3.0)
            self.no_found = True

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
    tag_detect = AprilTagDetectNode()
    tag_detect.pub_arm([90.0, 120.0, 0.0, 0.0, 90.0, 30.0])
    try:    
        rclpy.spin(tag_detect)
    except KeyboardInterrupt:
        pass
    finally:
        tag_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()













