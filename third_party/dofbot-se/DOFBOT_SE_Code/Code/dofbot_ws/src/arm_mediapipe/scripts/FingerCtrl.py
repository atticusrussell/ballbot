#!/usr/bin/env python3
# encoding: utf-8
import threading
import numpy as np
from media_library import *
from time import sleep, time

class HandCtrlArm:
    def __init__(self):
        self.media_ros = Media_ROS()
        self.hand_detector = HandDetector()
        self.arm_status = True
        self.locking = True
        self.init = True
        self.pTime = 0
        self.add_lock = self.remove_lock = 0
        self.media_ros.pub_arm([90, 135, 0, 45, 90, 90])
        print("111")
        self.event = threading.Event()
        self.event.set()

    def dance(self):
        print("dance")
        time_sleep = 0.5
        self.media_ros.pub_arm([90, 90, 90, 90, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 60, 120, 60, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 45, 135, 45, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 60, 120, 60, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 90, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 100, 80, 80, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 120, 60, 60, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 135, 45, 45, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 90, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 20, 90, 150])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 90, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 20, 90, 150])
        sleep(time_sleep)
        self.media_ros.pub_arm([0, 90, 90, 90, 0, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([0, 90, 180, 0, 0, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([], 6, 180)
        sleep(time_sleep)
        self.media_ros.pub_arm([], 6, 30)
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 90, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 135, 0, 45, 90, 90])
        sleep(time_sleep)

    def init_pose(self):
        self.media_ros.pub_arm([90.0, 145.0, 0.0, 0.0, 90.0, 31.0])
        sleep(0.5)
        self.media_ros.RobotBuzzer()
        

    def arm_applaud(self):
        print("applaud")
        self.media_ros.pub_arm([90.0, 145.0, 0.0, 71.0, 90.0, 0.0])
        sleep(0.5)
        self.media_ros.pub_arm([], 6, 30)
        sleep(0.5)
        self.media_ros.pub_arm([], 6, 180)
        sleep(0.5)
        self.media_ros.pub_arm([], 6, 30)
        sleep(0.5)
        self.media_ros.pub_arm([], 6, 180)
        sleep(0.5)
        #self.media_ros.pub_arm([90.0, 145.0, 0.0, 0.0, 90.0, 31.0])

    def shake(self):
        print("applaud")    
        for i in range(3):
            self.media_ros.pub_arm([138.0, 94.0, 92.0, 88.0, 92.0, 172.0])    
            sleep(0.5)
            self.media_ros.pub_arm([48.0, 94.0, 92.0, 87.0, 92.0, 172.0])    
            sleep(0.5)
        self.media_ros.pub_arm([90.0, 145.0, 0.0, 0.0, 90.0, 31.0])

    def arm_nod(self):
        print("nod")
        for i in range(3):
            self.media_ros.pub_arm([82.0, 89.0, 93.0, 93.0, 89.0, 32.0])    
            sleep(0.5)
            self.media_ros.pub_arm([82.0, 89.0, 93.0, 33.0, 89.0, 32.0])    
            sleep(0.5)
        self.media_ros.pub_arm([90.0, 145.0, 0.0, 0.0, 90.0, 31.0])

        
		
    def process(self, frame):
        frame = cv.flip(frame, 1)
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0 and self.media_ros.Joy_active:
            threading.Thread(target=self.arm_ctrl_threading, args=(lmList,bbox)).start()
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.media_ros.pub_imgMsg(frame)
        return frame

    def arm_ctrl_threading(self, lmList,bbox):
        if self.event.is_set():
            self.event.clear()
            fingers = self.hand_detector.fingersUp(lmList)
            self.hand_detector.draw = False
            gesture = self.hand_detector.get_gesture(lmList)
            if gesture == "Yes":	
                self.arm_status = False
                self.dance()
                sleep(0.5)
                self.init_pose()
                sleep(1.0)
                self.arm_status = True
			
            elif gesture == "OK":
                self.arm_status = False
                for i in range(3):
                    sleep(0.1)
                    self.media_ros.pub_arm([80, 135, 0, 15, 90, 180])
                    sleep(0.5)
                    self.media_ros.pub_arm([110, 135, 0, 15, 90, 90])
                    sleep(0.5)
                self.init_pose()
                sleep(1.0)
                self.arm_status = True
            elif gesture == "Thumb_down":
                self.arm_status = False

                self.media_ros.pub_arm([90, 0, 180, 0, 90, 180])
                sleep(1.0)
                self.media_ros.pub_arm([90, 0, 180, 0, 90, 90])
                sleep(1.0)
                self.init_pose()
                sleep(1.0)
                self.arm_status = True


            elif sum(fingers) == 1: 
                self.arm_status = False

                self.arm_nod()
                self.init_pose()
                sleep(1.0)
                self.arm_status = True


            elif fingers[1] == fingers[4] == 1 and sum(fingers) == 2: 
                self.arm_status = False

                self.shake()
                self.init_pose()
                sleep(1.0)
                self.arm_status = True

            elif sum(fingers) == 5: 
                self.arm_status = False
                self.arm_applaud()
                self.init_pose()
                sleep(1.0)
                self.arm_status = True

            self.event.set()

if __name__ == '__main__':
    rospy.init_node('HandCtrlArm_node', anonymous=True)
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    ctrl_arm = HandCtrlArm()
    ctrl_arm.media_ros.pub_arm([90, 135, 0, 45, 90, 90])
    while capture.isOpened():
        ret, frame = capture.read()
        action = cv.waitKey(1) & 0xFF
        frame = ctrl_arm.process(frame)
        if action == ord('q'):
            ctrl_arm.media_ros.cancel()
            break
        cv.imshow('frame', frame)
    capture.release()
    cv.destroyAllWindows()
