# !/usr/bin/env python3
# coding: utf-8
import math
import rospy
from dofbot_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse


class Control_Coord:
    def __init__(self):
        self.n = rospy.init_node('dofbot_control_coord', anonymous=True)
        # 创建用于调用的ROS服务的句柄 Create a handle to the ROS service to invoke
        self.client = rospy.ServiceProxy("dofbot_kinemarics", kinemarics)

    # 坐标转换成角度
    def coord_to_angles(self, coords):
        '''
        Post position request, get joint rotation angle
        发布位置请求,获取关节旋转角度
        '''
        # if len(coords) != 3:
        #     print("input error.coords=[x, y, z]")
        #     return False, None
        request = kinemaricsRequest()
        request.tar_x = coords[0]
        request.tar_y = coords[1]
        request.tar_z = coords[2]
        request.Roll  = coords[3]
        request.Pitch = coords[4]
        request.Yaw   = coords[5]
        request.kin_name = "ik"
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):
                # Get the inverse solution response result
                # 获得反解响应结果
                joints = [0.0, 0.0, 0.0, 0.0, 0.0]
                joints[0] = round(response.joint1, 2)
                joints[1] = round(response.joint2, 2)
                joints[2] = round(response.joint3, 2)
                joints[3] = round(response.joint4, 2)
                joints[4] = round(response.joint5, 2)
                return True, joints
        except Exception:
            rospy.loginfo("coord_to_angles error")
        return False, None

    # 角度转坐标
    def angles_to_coord(self, angles):
        '''
        Post position request, get joint rotation angle
        发布角度请求,获取坐标位置
        '''
        self.client.wait_for_service()
        request = kinemaricsRequest()
        request.cur_joint1 = angles[0]
        request.cur_joint2 = angles[1]
        request.cur_joint3 = angles[2]
        request.cur_joint4 = angles[3]
        request.cur_joint5 = angles[4]
        request.kin_name = "fk"
        Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        try:
            response = self.client.call(request)
            if isinstance(response, kinemaricsResponse):
                # 获得反解响应结果 Get the inverse solution response result
                Posture[0] = response.x
                Posture[1] = response.y
                Posture[2] = response.z
                Posture[3] = response.Roll
                Posture[4] = response.Pitch
                Posture[5] = response.Yaw
        except Exception:
            rospy.loginfo("angles_to_coord error")
        return Posture

