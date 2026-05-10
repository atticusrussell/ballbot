#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from std_msgs.msg import Float32, Bool
import time
import math
from dofbot_pro_interface.msg import *       # 需确认ROS2消息包名是否一致
from dofbot_pro_interface.srv import *
import transforms3d as tfs
import tf_transformations as tf         # ROS2使用tf_transformations
import threading
from ament_index_python import get_package_share_directory
import yaml
import os

class TagGraspNode1(Node):
    def __init__(self):
        super().__init__('test')
        self.sub = self.create_subscription(AprilTagInfo,'PosInfo',self.pos_callback,qos_profile=1)
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.client.wait_for_service(timeout_sec=5.0)
        self.request1 = Kinemarics.Request()
        # self.test()
        print("Init Done")     
        
    #颜色信息的回调函数，包括中心xy坐标和深度值z
    def test(self):
        self.request1.tar_x =0.1
        self.request1.tar_y = 0.02
        self.request1.tar_z = 0.3
        self.request1.kin_name = "ik"
        self.request1.roll = 0.08 
        # print("calcutelate_request: ",request1)
        self.current_future = self.client.call_async(self.request1)
        rclpy.spin_until_future_complete(self, self.current_future, timeout_sec=5.0) 
        # response = current_future.result()
        print("calcutelate_response: ",self.current_future.result())

    def pos_callback(self,msg):
        threading.Thread(target=self.test).start()

def main(args=None):
    rclpy.init(args=args)
    tag_grasp = TagGraspNode1()
    try:    
        rclpy.spin(tag_grasp)
    except KeyboardInterrupt:
        pass
    finally:
        tag_grasp.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
