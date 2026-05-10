#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2 as cv
import rospy
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from std_msgs.msg import Float32,Bool

import os
from cv_bridge import CvBridge
encoding = ['16UC1', '32FC1']
import time
import os
#color recognition
from astra_common import *
#from shape_recognize import *
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
import rospkg
from dofbot_pro_color.cfg import ColorHSVConfig
import math
from dofbot_pro_info.msg import *
from dofbot_pro_info.srv import *
import tf.transformations as tf
import transforms3d as tfs


class TagTrackingNode:
    def __init__(self):
        nodeName = 'color_detect'
        rospy.init_node(nodeName)
        self.init_joints = [90.0, 120, 0.0, 0.0, 90, 90]
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw',Image)
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.TimeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub,self.depth_image_sub],10,0.5)
        self.TimeSynchronizer.registerCallback(self.ComputeVolume)
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()

        #color
        self.Roi_init = ()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.dyn_update = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'init'
        self.color = color_detect()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.hsv_text = rospkg.RosPack().get_path("dofbot_pro_color") + "/scripts/colorHSV.text"
        Server(ColorHSVConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client(nodeName, timeout=10)
        self.cx = 0
        self.cy = 0
        self.error = False
        self.circle_r = 0 #防止误识别到其他的杂乱的点
        self.CurEndPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.90000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,4.90000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.get_current_end_pos()
        self.get_corner = [0,0,0,0,0,0]
        exit_code = os.system('rosservice call /camera/set_color_exposure  50')

       

        
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
        
        
        
    def GraspStatusCallback(self,msg):
        if msg.data == True:
            self.pubPos_flag = True

    def dynamic_reconfigure_callback(self, config, level):
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        write_HSV(self.hsv_text, self.hsv_range)
        return config     

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
    
    def ComputeVolume(self,color_frame,depth_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'bgr8')
        result_image = np.copy(rgb_image)
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        action = cv.waitKey(10) & 0xFF
        result_image = cv.resize(result_image, (640, 480))
        result_frame, binary= self.process(result_image,action)
        if self.color.shape_cx!=0 and self.color.shape_cy!=0:
            
            if self.color.shape_name == "Rectangle" or self.color.shape_name == "Cylinder":
                x1 = self.get_corner[0]
                y1 = self.get_corner[1]
                z1 = depth_image_info[y1,x1]/1000
                if z1!=0:
                    c1_pose_T = self.get_pos(x1,y1,z1)
                else:
                    self.error = True
                    print("z1 invalid Distance!")
                    
                x2 = self.get_corner[2]
                y2 = self.get_corner[3]
                z2 = depth_image_info[y2,x2]/1000
                if z1!=0:
                    c2_pose_T = self.get_pos(x2,y2,z2)
                else:
                    self.error = True
                    print("z2 invalid Distance!")
                
                
                x3 = self.get_corner[4]
                y3 = self.get_corner[5]
                z3 = depth_image_info[y3,x3]/1000
                if z1!=0:
                    c3_pose_T = self.get_pos(x3,y3,z3)
                else:
                    self.error = True
                    print("z3 invalid Distance!") 
    
                cx = self.color.shape_cx
                cy = self.color.shape_cy
                cz = depth_image_info[cy,cx]/1000
                if cz!=0:
                    print("cz: ",cz)
                    center_pose_T = self.get_pos(cx,cy,cz)
                    height = center_pose_T[2]*100
                    print("get height: ",height)

                if self.error!=True:
                    # 定义两个点的坐标
                    point1 = np.array([c1_pose_T[0], c1_pose_T[1], c1_pose_T[2]])
                    point2 = np.array([c2_pose_T[0], c2_pose_T[1], c2_pose_T[2]])
                    point3 = np.array([c3_pose_T[0], c3_pose_T[1], c3_pose_T[2]])
                    c_ponit = np.array([center_pose_T[0], center_pose_T[1], center_pose_T[2]])
                    r = np.linalg.norm(point1 - c_ponit)*100  
                    # 计算欧几里得距离
                    distance1 = np.linalg.norm(point1 - point3)*100
                    distance2 = np.linalg.norm(point3 - point2)*100

                   
                    if self.color.shape_name == "Rectangle":
                        print("shape_name: ",self.color.shape_name)
                        print("distance1: ",distance1)
                        print("distance2: ",distance2)
                        print("height: ",height)
                        volume = distance1 * distance2 * height
                        print("volume: ",format(volume, 'f'))
                        print("---------------------------")
                    if self.color.shape_name == "Cylinder":
                        print("r: ",r)
                        print("height: ",height)
                        print("shape_name: ",self.color.shape_name)
                        volume = math.pi*r*r* height
                        print("volume: ",format(volume, 'f'))
                        print("---------------------------")
                        
            if self.color.shape_name == "Square":
                print("shape_name: ",self.color.shape_name)
                cx = self.color.shape_cx
                cy = self.color.shape_cy
                cz = depth_image_info[cy,cx]/1000
                if cz!=0:
                    center_pose_T = self.get_pos(cx,cy,cz)
                    height = center_pose_T[2]*100
                    print("height: ",height)
                    volume = height * height * height
                    
                    print("volume: ",format(volume, 'f'))
                    print("---------------------------") 
   
            self.error = False
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([result_frame, binary])))
        else:
            cv.imshow(self.windows_name, result_frame)


    def process(self, rgb_img, action):
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == 32: self.pubPos_flag = True
        elif action == ord('i') or action == ord('I'): self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): self.Reset()
        #elif action == ord('q') or action == ord('Q'): self.cancel()
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text): self.hsv_range = read_HSV(self.hsv_text)
            else: self.Track_state = 'init'
        if self.Track_state != 'init':
            if len(self.hsv_range) != 0:
                rgb_img, binary, self.circle,corner = self.color.ShapeRecognition(rgb_img, self.hsv_range)
                self.get_corner = corner
                #print("corner: ",corner)
                #cv.circle(rgb_img, (coner[3][0],coner[3][1]), 3, (0,255,255), thickness=1)
                if self.dyn_update == True:
                    write_HSV(self.hsv_text, self.hsv_range)
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                    self.dyn_client.update_configuration(params)
                    self.dyn_update = False

        return rgb_img, binary

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        self.init_joints = [90.0, 93, 37, 0.0, 90, 90]
        #self.dofbot_tracker.pub_arm(self.init_joints)
        self.cx = 0
        self.cy = 0
        self.pubPos_flag = False
        



    def pub_arm(self, joints, id=6, angle=180, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        if len(joints) != 0: arm_joint.joints = joints
        else: arm_joint.joints = []
        self.pubPoint.publish(arm_joint)
        
        
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
    
    def get_pos(self,x,y,z):
        camera_location = self.pixel_to_camera_depth((x,y),z)
        
        #print("camera_location: ",camera_location)
        PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
        #PoseEndMat = np.matmul(self.xyz_euler_to_mat(camera_location, (0, 0, 0)),self.EndToCamMat)
        EndPointMat = self.get_end_point_mat()
        WorldPose = np.matmul(EndPointMat, PoseEndMat) 
        #WorldPose = np.matmul(PoseEndMat,EndPointMat)
        pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
        return pose_T
        

if __name__ == '__main__':
    try:
        tag_tracking = TagTrackingNode()
        tag_tracking.pub_arm(tag_tracking.init_joints)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

