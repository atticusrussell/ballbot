#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from std_msgs.msg import Float32,Bool,Int8

import os
from cv_bridge import CvBridge
import cv2 as cv

encoding = ['16UC1', '32FC1']
import time

#color recognition
#from astra_common import *
from height_measurement import *
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
import rospkg
from dofbot_pro_color.cfg import ColorHSVConfig
import math
from dofbot_pro_info.msg import *
from dofbot_pro_info.srv import *
import transforms3d as tfs
import tf.transformations as tf
class ColorDetectNode:
    def __init__(self):
        nodeName = 'get_color_heigh_list'
        rospy.init_node(nodeName)
        self.init_joints = [90.0, 120, 0.0, 0.0, 90, 90]

        self.pub_ColorInfo = rospy.Publisher("xyz", Position, queue_size=1)
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.sub_voice = rospy.Subscriber('voice_result', Int8, self.getVoiceResultCallback, queue_size=1)
        self.grasp_status_sub = rospy.Subscriber('grasp_done', Bool, self.GraspStatusCallback, queue_size=1)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw',Image)
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.client = rospy.ServiceProxy("get_kinemarics", kinemarics)
        self.TimeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub,self.depth_image_sub],10,0.5)
        self.TimeSynchronizer.registerCallback(self.Color_Detect)
        self.pr_time = time.time()
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.pubPos_flag = True
        self.start_flag = False
        self.heigh = 0.0
        self.CurEndPos = [-0.006,0.116261662208,0.0911289015753,-1.04719,-0.0,0.0]
        self.camera_info_K = [477.57421875, 0.0, 319.3820495605469, 0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[1.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00],
                                     [0.00000000e+00,7.96326711e-04,9.99999683e-01,-9.90000000e-02],
                                     [0.00000000e+00,-9.99999683e-01,7.96326711e-04,4.90000000e-02],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.get_current_end_pos()

        #color
        self.CX_list = []
        self.CY_list = []
        self.Roi_init = ()
        self.hsv_range = ()
        self.point_pose = (0, 0, 0)
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
        self.dyn_client = Client(nodeName, timeout=60)
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
            
    def getVoiceResultCallback(self,msg):   
        if msg.data == 11:
            self.start_flag = True

    
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

    
    def Color_Detect(self,color_frame,depth_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'bgr8')
        result_image = np.copy(rgb_image)
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        depth_to_color_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        action = cv.waitKey(10) & 0xFF
        result_image = cv.resize(result_image, (640, 480))
        result_frame, binary = self.process(result_image,action)
        print("self.CX_list: ",self.CX_list)
        print("self.CY_list: ",self.CY_list)
        if self.pubPos_flag == True:
            for i in range(len(self.CX_list)):
                
                cx = self.CX_list[i]
                cy = self.CY_list[i]
                cv.circle(depth_to_color_image,(int(cx),int(cy)),1,(255,255,255),10)
                dist = depth_image_info[cy,cx]/1000
                heigh = round(self.compute_heigh(cx,cy,dist),2) *1000
                print("heigh: ",heigh)
                heigh_msg = str(heigh) + 'mm'
                cv.putText(result_frame, heigh_msg, (cx+5, cy+5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2) 

                if heigh>40:
                    pos = Position()
                    pos.x = cx
                    pos.y = cy
                    pos.z = dist
                    print("pos.z: ",pos.z)
                    if pos.z!=0 and self.start_flag==True:
                        self.pub_ColorInfo.publish(pos)
                        self.pubPos_flag = False
                           
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_frame, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)   
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([result_frame, binary])))
        else:
            cv.imshow(self.windows_name, result_frame)
        cv2.imshow("depth_image", depth_to_color_image)


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
                rgb_img, binary, self.CX_list,self.CY_list = self.color.ShapeRecognition(rgb_img, self.hsv_range)
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
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        self.init_joints = [90.0, 93, 37, 0.0, 90, 90]
        #self.dofbot_tracker.pub_arm(self.init_joints)
        self.pubPos_flag = False
        

    def calculate_yaw(self,bin_img):
        contours = cv.findContours(bin_img, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)[-2]
        c = max(contours, key = cv.contourArea)
        area = math.fabs(cv.contourArea(c))
        rect = cv.minAreaRect(c)
        

    def pub_arm(self, joints, id=6, angle=180, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        if len(joints) != 0: arm_joint.joints = joints
        else: arm_joint.joints = []
        self.pubPoint.publish(arm_joint)
        
    def compute_heigh(self,x,y,z):
        camera_location = self.pixel_to_camera_depth((x,y),z)
        #print("camera_location: ",camera_location)
        PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
        #PoseEndMat = np.matmul(self.xyz_euler_to_mat(camera_location, (0, 0, 0)),self.EndToCamMat)
        EndPointMat = self.get_end_point_mat()
        WorldPose = np.matmul(EndPointMat, PoseEndMat) 
        #WorldPose = np.matmul(PoseEndMat,EndPointMat)
        pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
        #print("pose_T: ",pose_T)
        return pose_T[2]
        

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

if __name__ == '__main__':
    try:
        color_detect = ColorDetectNode()
        color_detect.pub_arm(color_detect.init_joints)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

