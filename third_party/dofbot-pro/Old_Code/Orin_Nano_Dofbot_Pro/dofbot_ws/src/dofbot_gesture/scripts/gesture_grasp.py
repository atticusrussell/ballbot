#!/usr/bin/env python3
# encoding: utf-8
import cv2 as cv
import time
from dofbot_utils.GestureRecognition import handDetector
import sys                                                                  
from dofbot_utils.fps import FPS
from dofbot_utils.robot_controller import Robot_Controller
from Arm_Lib import Arm_Device
import threading


class Gesture_Grasp:
    def __init__(self):
        self.hand_detector = handDetector(detectorCon=0.75)
        self.pTime = 0

        # 定义手势识别次数
        self.Count_One = 0
        self.Count_Two = 0
        self.Count_Three = 0
        self.Count_Four = 0
        self.Count_Five = 0

        self.arm = Arm_Device()
        self.move_state = False
        self.fps = FPS()
        self.robot = Robot_Controller()
        self.grap_joint = self.robot.get_gripper_value(1)
        self._joint_5 = self.robot.joint5

        self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_AT, 1000)


    def ctrl_arm_move(self, index):
        if index == 5:
            for i in range(5):
                self.arm.Arm_serial_servo_write(5, 60, 300)
                time.sleep(0.4)
                self.arm.Arm_serial_servo_write(5, 120, 300)
                time.sleep(0.4)
            self.arm.Arm_serial_servo_write(5, 90, 300)
            time.sleep(0.4)
            return

        # raise 抬起
        joints_uu = [90, 80, 50, 50, self._joint_5, 30]
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1500)
        time.sleep(2)
        # Move to object position 移动至物体位置
        if index == 1:
            joints_num = self.robot.P_NUM_1
            joints_num[5] = self.robot.get_gripper_value(0)
            self.arm.Arm_serial_servo_write6_array(joints_num, 1000)
        elif index == 2:
            joints_num = self.robot.P_NUM_2
            joints_num[5] = self.robot.get_gripper_value(0)
            self.arm.Arm_serial_servo_write6_array(joints_num, 1000)
        elif index == 3:
            joints_num = self.robot.P_NUM_3
            joints_num[5] = self.robot.get_gripper_value(0)
            self.arm.Arm_serial_servo_write6_array(joints_num, 1000)
        elif index == 4:
            joints_num = self.robot.P_NUM_4
            joints_num[5] = self.robot.get_gripper_value(0)
            self.arm.Arm_serial_servo_write6_array(joints_num, 1000)
        time.sleep(1.5)
        # Grasp and clamp the clamping claw进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        time.sleep(1)
        # put up 架起
        self.arm.Arm_serial_servo_write(2, 70, 1000)
        time.sleep(1)
        joints_up = [90, 80, 50, 50, self._joint_5, self.grap_joint]
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        time.sleep(1.5)
        # Move to target location 移动至目标位置
        self.arm.Arm_serial_servo_write6_array(self.robot.P_CENTER, 1000)
        time.sleep(1.5)
        # Release the object and release the clamping jaws释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        time.sleep(1)
        # raise  抬起, 恢复到默认姿态
        self.arm.Arm_serial_servo_write6_array(self.robot.P_LOOK_AT, 1300)
        time.sleep(1.5)
        

    def process(self, frame):
        frame, lmList = self.hand_detector.findHands(frame, draw=False)
        if len(lmList) != 0:
            gesture = self.hand_detector.get_gesture()
            if gesture == 'One':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_One = self.Count_One + 1
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Four = 0
                self.Count_Five = 0
                if self.Count_One >= 10  and self.move_state == False:
                    self.move_state = True
                    # print("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()
            elif gesture == 'Two':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Two = self.Count_Two + 1
                self.Count_Three = 0
                self.Count_Four = 0
                self.Count_Five = 0
                if self.Count_Two >= 10  and self.move_state == False:
                    if not self.move_state:
                        self.move_state = True
                        # print("start arm_ctrl_threading = {}".format(gesture))
                        task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                        task.setDaemon(True)
                        task.start()
            elif gesture == 'Three':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Three = self.Count_Three + 1
                self.Count_Two = 0
                self.Count_Four = 0
                self.Count_Five = 0
                if self.Count_Three >= 10  and self.move_state == False:
                    self.move_state = True
                    # print("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()
            elif gesture == 'Four':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Four = self.Count_Four + 1
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Five = 0
                if self.Count_Four >= 10  and self.move_state == False:
                    self.move_state = True
                    # print("start arm_ctrl_threading = {}".format(gesture))
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
                if self.Count_Five >= 10  and self.move_state == False:
                    self.Count_Five = 0
                    self.move_state = True
                    # print("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()

        self.fps.update_fps()
        self.fps.show_fps(frame)
        return frame
        


    def arm_ctrl_threading(self, gesture):
        # print("arm_ctrl_threading gesture = {}".format(gesture))
        if gesture == 'One':
            self.ctrl_arm_move(1)
        elif gesture == 'Two':
            self.ctrl_arm_move(2)
        elif gesture == 'Three':
            self.ctrl_arm_move(3)
        elif gesture == 'Four':
            self.ctrl_arm_move(4)
        elif gesture == 'Five':
            self.ctrl_arm_move(5)
        self.move_state = False



if __name__ == '__main__':
    capture = cv.VideoCapture(0)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    gesture = Gesture_Grasp()
    while capture.isOpened():
        try:
            ret, frame = capture.read()
            action = cv.waitKey(1) & 0xFF
            frame = gesture.process(frame)
            if action == ord('q'):
                break
            cv.imshow('frame', frame)
        except:
            print("break")
            break 
    print("capture.release()")
    capture.release()
    cv.destroyAllWindows()
