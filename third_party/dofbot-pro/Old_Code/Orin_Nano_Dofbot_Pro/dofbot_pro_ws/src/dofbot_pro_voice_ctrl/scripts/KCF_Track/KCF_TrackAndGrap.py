#!/usr/bin/env python
# -*- coding: utf-8 -*-
print("语音指令词如下：")
print("------追踪夹取物体------")
import rospy
import numpy as np
from std_msgs.msg import Float32,Int8
from Dofbot_Track import *
from dofbot_info.srv import kinemarics, kinemaricsRequest, kinemaricsResponse
from dofbot_pro_info.msg import *
import message_filters
encoding = ['16UC1', '32FC1']
import time
import os

class KCFTrackingNode:
    def __init__(self):
        rospy.init_node('KCF_tracking')
        self.start_flag=False
        self.dofbot_tracker = DofbotTrack()
        self.init_joints = [90.0, 150, 12, 20.0, 90, 3]
        self.pos_sub = message_filters.Subscriber('/pos_xyz',Position)
        self.sub_voice = rospy.Subscriber("voice_result",Int8,self.getVoiceResultCallBack)
        self.TimeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.pos_sub],10,0.5,allow_headerless=True)
        self.TimeSynchronizer.registerCallback(self.TrackAndGrap)
        self.cur_distance = 0.0
        self.cnt = 0
        exit_code = os.system('rosservice call /camera/set_color_exposure  50')
        print("Init done!")       
    
    def TrackAndGrap(self,position):

        #print("position_info: ",position)
        
        if self.start_flag == True :

            center_x, center_y = position.x,position.y

            self.cur_distance = position.z
            
            if abs(center_x-320) >2 or abs(center_y-240)>2:
                self.dofbot_tracker.XY_track(center_x,center_y)         
            else:
                self.cnt = self.cnt + 1
                if self.cnt == 10 :
                    self.cnt = 0 
                    if self.cur_distance!= 999 :
                        self.dofbot_tracker.stop_flag = True
                        self.dofbot_tracker.Clamping(center_x,center_y,self.cur_distance)

    def getVoiceResultCallBack(self,msg):
        if msg.data == 14:
            self.start_flag = True
            print("Start tracking and grabbing.")
                            
if __name__ == '__main__':
    try:
        kcf_tracking = KCFTrackingNode()
        kcf_tracking.dofbot_tracker.pub_arm(kcf_tracking.init_joints)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
