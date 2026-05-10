#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from largemodel_arm.Dofbot_Track import *
from dofbot_pro_interface.msg import *
from std_msgs.msg import String,Int16

class KCFTrackingNode(Node):
    def __init__(self):
        super().__init__('KCF_tracking')
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
        self.dofbot_tracker = DofbotTrack()        
        self.pos_sub = Subscriber(self,Position,'/pos_xyz')
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", 10)
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',1)
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')


        
        self.time_sync = ApproximateTimeSynchronizer([self.pos_sub],queue_size=10,slop =0.1,allow_headerless=True)
        self.time_sync.registerCallback(self.TrackAndGrap)
        self.CurEndPos = [-0.000599999999999989,0.11626166220790028,0.09112890157533887,-1.0471975309176935,-0.0,0.0]
        self.cur_distance = 0.0
        self.cnt = 0
        self.done = True
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            self.get_current_end_pos()
           
        while not self.pubPoint.get_subscription_count():
            self.pub_arm(self.init_joints)
            time.sleep(0.1) 
        self.pub_arm(self.init_joints)
        
        print("Init done!")
    
    def TrackAndGrap(self,position):  
        center_x, center_y = position.x,position.y
        self.cur_distance = position.z
        if self.done == True:
            self.get_current_end_pos() 
            self.get_logger().info('-*-*-*-*-*-*-*-*-*-*')
            print("self.CurEndPos: ",self.CurEndPos)
            self.dofbot_tracker.Clamping(center_x,center_y,self.cur_distance,self.CurEndPos)
            self.done = False



    def pub_arm(self, joints, id=6, angle=180.0, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        arm_joint.joints = joints
        self.pubPoint.publish(arm_joint)


    def get_current_end_pos(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')	
        request = Kinemarics.Request()
        request.cur_joint1 = self.init_joints[0]
        request.cur_joint2 = self.init_joints[1]
        request.cur_joint3 = self.init_joints[2]
        request.cur_joint4 = self.init_joints[3]
        request.cur_joint5 = self.init_joints[4]
        request.kin_name = "fk"
        future = self.client.call_async(request)
        future.add_done_callback(self.get_fk_respone_callback)
        return True

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

def main(args=None):
    rclpy.init(args=args)
    kcf_tracking = KCFTrackingNode()
    try:    
        rclpy.spin(kcf_tracking)
    except KeyboardInterrupt:
        pass
    finally:
        kcf_tracking.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 

