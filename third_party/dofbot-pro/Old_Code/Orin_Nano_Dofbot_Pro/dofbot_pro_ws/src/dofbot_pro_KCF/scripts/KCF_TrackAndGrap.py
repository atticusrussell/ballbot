#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import message_filters
from Dofbot_Track import *
from dofbot_pro_info.msg import *
import os

class KCFTrackingNode:
    def __init__(self):
        rospy.init_node('KCF_tracking')
        self.dofbot_tracker = DofbotTrack()
        self.pos_sub = message_filters.Subscriber('/pos_xyz',Position)
        self.TimeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.pos_sub],10,0.5,allow_headerless=True)
        self.TimeSynchronizer.registerCallback(self.TrackAndGrap)
        self.cur_distance = 0.0
        self.cnt = 0
        exit_code = os.system('rosservice call /camera/set_color_exposure  50')
        print("Init done!")
                        
    
    def TrackAndGrap(self,position):  
        center_x, center_y = position.x,position.y
        self.cur_distance = position.z
        if abs(center_x-320) >10 or abs(center_y-240)>10 :
            self.dofbot_tracker.XY_track(center_x,center_y)         
        else:
            self.cnt = self.cnt + 1
            if self.cnt == 10 :
                self.cnt = 0
                if self.cur_distance!= 999:
                    self.dofbot_tracker.stop_flag = True
                    self.dofbot_tracker.Clamping(center_x,center_y,self.cur_distance)
                            
if __name__ == '__main__':
    try:
        kcf_tracking = KCFTrackingNode()
        kcf_tracking.dofbot_tracker.pub_arm(kcf_tracking.dofbot_tracker.init_joints)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
