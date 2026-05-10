#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from dofbot_pro_KCF.Dofbot_Track import *
from dofbot_pro_interface.msg import *
from std_msgs.msg import String

class KCFTrackingNode(Node):
    def __init__(self):
        super().__init__('KCF_tracking')
        self.dofbot_tracker = DofbotTrack()
        self.pos_sub = Subscriber(self,Position,'/pos_xy')
        self.sub_grasp_status = self.create_subscription(Position,"/pos_xy",self.TrackAndGrap,100)
        self.cur_distance = 0.0
        self.cnt = 0
        print("Init done!")

    
    def TrackAndGrap(self,position):  
        center_x, center_y = position.x,position.y
        if abs(center_x-320) >8 or abs(center_y-240)>8 :
            self.dofbot_tracker.XY_track(center_x,center_y)  


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

