#!/usr/bin/env python3
# encoding: utf-8
import threading
import numpy as np
from media_library import *
from time import sleep, time
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from dofbot_pro_info.msg import *
import os
class DetectGesture:
    def __init__(self):
        self.media_ros = Media_ROS()
        self.hand_detector = HandDetector()
        self.pTime = 0
        self.img_sub = rospy.Subscriber("/image_data",Image_Msg,self.image_sub_callback)
        self.graspStatus_sub = rospy.Subscriber("grasp_done",Bool,self.graspStatusCallBack)
        self.pubPoint = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)
        self.pub_targetID = rospy.Publisher("TargetId",Int8,queue_size=1)
        self.detect_gesture = Int8()
        self.pTime = self.cTime = 0
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像
        self.cnt = 0
        self.last_sum = 0
        self.detect_gesture_joints = [90,150,12,20,90,30]
        self.pub_gesture = True
        exit_code = os.system('rosservice call /camera/set_color_exposure  50')
        
        
    def pubTargetArm(self, joints, id=6, angle=180, runtime=2000):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        if len(joints) != 0: arm_joint.joints = joints
        else: arm_joint.joints = []
        self.pubPoint.publish(arm_joint)      
                
        
    def graspStatusCallBack(self,msg):
        if msg.data == True:
            print("Publish the next gesture")
            self.pub_gesture = True    
            self.cnt = 0
        		
    def process(self, frame):
        #frame = cv.flip(frame, 1)
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0:
            threading.Thread(target=self.Gesture_Detect_threading, args=(lmList,bbox)).start()
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        #self.media_ros.pub_imgMsg(frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            rospy.signal_shutdown("exit")
        cv.imshow('frame', frame)

    def Gesture_Detect_threading(self, lmList,bbox):
        fingers = self.hand_detector.fingersUp(lmList)
        #print("sum of fingers: ",sum(fingers))
        self.last_sum = sum(fingers)
        print(self.pub_gesture)
        if sum(fingers) == self.last_sum:
            print("---------------------------")
            self.cnt = self.cnt + 1
            print("cnt: ",self.cnt)
            if self.cnt==30 and self.pub_gesture == True:
                print("sum of fingers: ",self.last_sum)
                self.pub_gesture = False
                self.detect_gesture.data = self.last_sum   
                self.pub_targetID.publish(self.detect_gesture)
                self.last_sum = 0
                self.cnt = 0
        
    def image_sub_callback(self,msg):
        image = np.ndarray(shape=(msg.height, msg.width, msg.channels), dtype=np.uint8, buffer=msg.data) # 将自定义图像消息转化为图像
        self.img[:,:,0],self.img[:,:,1],self.img[:,:,2] = image[:,:,2],image[:,:,1],image[:,:,0] # 将rgb 转化为opencv的bgr顺序
        frame = self.img.copy()
        self.process(frame)
        

if __name__ == '__main__':
    rospy.init_node('Detect_Gesture_node', anonymous=True)
    detect_gesture = DetectGesture()
    detect_gesture.pubTargetArm(detect_gesture.detect_gesture_joints)
    rospy.spin()
