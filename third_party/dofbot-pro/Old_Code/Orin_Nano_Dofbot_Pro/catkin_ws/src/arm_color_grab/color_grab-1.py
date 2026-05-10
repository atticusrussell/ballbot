#!/usr/bin/env python3
# encoding: utf-8
import cv2 as cv
import rospy
import threading
from dofbot_utils.robot_controller import Robot_Controller
from dofbot_utils.color_recognition import ColorRecognition
from dofbot_utils.dofbot_config import *
from Arm_Lib import Arm_Device
import sys
import time
import signal


class color_grab:
    def __init__(self):
        self.event = threading.Event()
        self.event.set()
        self.pTime = 0
        self.colorRecognition = ColorRecognition()
        self.robot = Robot_Controller()
        self.color_hsv = {"red": ((0, 103, 172), (2, 255, 255)),
                          "green": ((54, 109, 78), (77, 255, 255)),
                          "blue": ((92, 100, 62), (121, 251, 255)),
                          "yellow": ((26, 100, 91), (32, 255, 255))}
        # HSV参数路径  HSV Parameter path
        self.HSV_path = "/home/jetson/dofbot_ws/src/dofbot_grasp/scripts/HSV_config_ahead.txt"

        # 定义抓取方块的状态
        self.yellow_grabbed = 0
        self.red_grabbed = 0
        self.green_grabbed = 0
        self.blue_grabbed = 0
        self.num = 0
        self.status = 'waiting'
        self.last_color = None
        self.Arm = Arm_Device()
        read_HSV(self.HSV_path, self.color_hsv)
        self.robot.move_init_pose()

    def reset_state(self):
        pass
        
    def move_action(self, index):
        self.robot.arm_move(self.robot.P_TOP, 1000)
        time.sleep(1.5)
        if index == 4:
            # 抓取黄色的积木块
            self.robot.arm_move(self.robot.P_YELLOW, 1500)
            time.sleep(2)
        elif index == 3:
            # 抓取红色的积木块
            self.robot.arm_move(self.robot.P_RED, 1500)
            time.sleep(2)
        elif index == 2:
            # 抓取绿色的积木块
            self.robot.arm_move(self.robot.P_GREEN, 1500)
            time.sleep(2)
        elif index == 1:
            # 抓取蓝色的积木块
            self.robot.arm_move(self.robot.P_BLUE, 1500)
            time.sleep(2)
        self.robot.arm_clamp_block(1)
        time.sleep(1)
        self.robot.arm_move(self.robot.P_TOP, 1000)
        time.sleep(1.5)

        self.robot.arm_move(self.robot.P_CENTER, 1500)
        time.sleep(2)
        self.robot.arm_clamp_block(0)
        time.sleep(1)
        self.robot.arm_move_6(self.robot.P_LOOK_AT, 1000)
        time.sleep(1)

    

    def grasp_run(self, color_name):
        self.robot.arm_clamp_block(0)
        time.sleep(.1)
        self.Arm.Arm_Buzzer_On(1)
        time.sleep(.5)
        if color_name == 'yellow':
            print("yellow")  
            self.move_action(4)
        elif color_name == 'red':
            print("red")
            self.move_action(3)
        elif color_name == 'green':
            print("green")
            self.move_action(2)
        elif color_name == 'blue':
            print("blue")
            self.move_action(1)

        self.status = 'waiting'


    def start_grab(self, frame):
        if self.status == 'waiting':
            frame, color = self.colorRecognition.get_all_color(frame, self.color_hsv)
            if len(color) == 0:
                color_name = None
            else:
                color_name = color[0]
            if len(color) == 1 and color_name == self.last_color:
                self.num += 1
                if self.num > 10 and self.status == 'waiting':
                    self.status = "running"
                    self.num = 0
                    task = threading.Thread(target=self.grasp_run, name="grasp_run", args=(color_name, ))
                    task.setDaemon(True)
                    task.start()
                    # self.grasp_run(color_name)
            else:
                self.num = 0
                self.last_color = color_name

        self.cTime = time.time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30),
                    cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        return frame


def quit(signum, frame):
    print("sys.exit")
    sys.exit()

