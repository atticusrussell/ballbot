#!/usr/bin/env python3
# encoding: utf-8
import cv2 as cv
import rospy
import time
from GestureRecognition import handDetector
import sys                                                                  
from dofbot_utils.robot_controller import Robot_Controller
from Arm_Lib import Arm_Device
import threading


class Gesture_Action:
    def __init__(self):
        self.hand_detector = handDetector(detectorCon=0.75)
        self.pTime = 0

        # 定义抓取方块的状态
        self.one_grabbed = 0
        self.two_grabbed = 0
        self.three_grabbed = 0
        self.four_grabbed = 0

        self.block_num = 0

        # 定义手势识别次数
        self.Count_One = 0
        self.Count_Two = 0
        self.Count_Three = 0
        self.Count_Four = 0
        self.Count_Five = 0

        self.arm = Arm_Device()
        self.move_state = False
        self.robot = Robot_Controller()
        self.grap_joint = self.robot.get_gripper_value(1)
        self._joint_5 = self.robot.joint5

        self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_AT, 1000)


    def process(self, frame):
        frame, lmList = self.hand_detector.findHands(frame, draw=False)
        if len(lmList) != 0:
            gesture = self.hand_detector.get_gesture()
            # rospy.loginfo("gesture = {}".format(gesture))
            
            if gesture == 'One':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_One = self.Count_One + 1
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Four = 0
                self.Count_Five = 0
                if self.Count_One >= 10 and self.move_state == False:
                    self.move_state = True
                    self.Count_One = 0
                    rospy.loginfo("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()

            elif gesture == 'Two':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Two = self.Count_Two + 1
                self.Count_Three = 0
                self.Count_Four = 0
                self.Count_Five = 0
                if self.Count_Two >= 10 and self.move_state == False:
                    self.move_state = True
                    self.Count_Two = 0
                    rospy.loginfo("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()

            elif gesture == 'Three':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Three = self.Count_Three + 1
                self.Count_Two = 0
                self.Count_Four = 0
                self.Count_Five = 0
                if self.Count_Three >= 10 and self.move_state == False:
                    self.move_state = True
                    self.Count_Three = 0
                    rospy.loginfo("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()
                    
            elif gesture == 'Four':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Four = self.Count_Four + 1
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Five = 0
                if self.Count_Four >= 10 and self.move_state == False:
                    self.move_state = True
                    self.Count_Four = 0
                    rospy.loginfo("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()

            elif gesture == 'Five':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Five = self.Count_Five + 1
                self.Count_One = 0
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Four = 0
                if self.Count_Five >= 10 and self.move_state == False:
                    self.move_state = True
                    self.Count_Five = 0
                    rospy.loginfo("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()
        return frame
        
    def reset_state(self):
        self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_AT, 1000)
        time.sleep(1)
        
    
    def arm_ctrl_threading(self, gesture):
        rospy.loginfo("arm_ctrl_threading gesture = {}".format(gesture))
        if gesture == 'One':
            self.arm.Arm_serial_servo_write6_array(self.robot.P_ACTION_1, 1000)
            time.sleep(1.5)
            self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_AT, 1000)
            time.sleep(1)
        elif gesture == 'Two':
            self.arm.Arm_serial_servo_write6_array(self.robot.P_ACTION_2, 1000)
            time.sleep(1.5)
            for i in range(5):
                self.arm.Arm_serial_servo_write(6, 180, 100)
                time.sleep(0.15)
                self.arm.Arm_serial_servo_write(6, 30, 100)
                time.sleep(0.15)
            self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_AT, 1000)
            time.sleep(1)
        elif gesture == 'Three':
            for i in range(3):
                self.arm.Arm_serial_servo_write6_array(self.robot.P_ACTION_3, 1200)
                time.sleep(1.2)
                self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_AT, 1000)
                time.sleep(1)
        elif gesture == 'Four':
            self.arm.Arm_serial_servo_write6_array(self.robot.P_ACTION_4, 1500)
            time.sleep(1.4)
            for i in range(3):
                self.arm.Arm_serial_servo_write(4, -15, 300)
                time.sleep(0.4)
                self.arm.Arm_serial_servo_write(4, 20, 300)
                time.sleep(0.4)
            self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_AT, 1000)
            time.sleep(1)
        elif gesture == 'Five':
            for i in range(5):
                self.arm.Arm_serial_servo_write(5, 60, 300)
                time.sleep(0.4)
                self.arm.Arm_serial_servo_write(5, 120, 300)
                time.sleep(0.4)
            self.arm.Arm_serial_servo_write(5, 90, 300)
            time.sleep(0.4)
            self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_AT, 1000)
            time.sleep(1)
        self.move_state = False




if __name__ == '__main__':
    rospy.init_node('gesture_recognition_stacking', anonymous=True)
    capture = cv.VideoCapture(0)
    # capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    gestureRecognitionStacking = Gesture_Action()
    while capture.isOpened():
        try:
            ret, frame = capture.read()
            action = cv.waitKey(1) & 0xFF
            frame = gestureRecognitionStacking.process(frame)
            if action == ord('q'):
                break
            cv.imshow('frame', frame)
        except:
            rospy.loginfo("break")
            break 
    rospy.loginfo("capture.release()")
    capture.release()
    cv.destroyAllWindows()
