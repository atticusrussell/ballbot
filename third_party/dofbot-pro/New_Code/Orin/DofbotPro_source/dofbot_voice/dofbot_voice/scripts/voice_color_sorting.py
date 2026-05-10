#!/usr/bin/env python
# coding: utf-8
from Speech_Lib import Speech
mySpeech = Speech()

import sys
sys.path.append('/home/jetson/dofbot_pro/dofbot_color_sorting/scripts')

import cv2 as cv
import threading
from time import sleep

import ipywidgets as widgets
from IPython.display import display
from speech_identify_target import identify_GetTarget
from dofbot_utils.fps import FPS
from dofbot_utils.dofbot_config import *

## 创建获取目标实例
target      = identify_GetTarget()
# 创建相机标定实例
calibration = Arm_Calibration()
# 初始化一些参数
num=0
dp    = []
xy=[90,106]
msg   = {}
threshold = 116
model = "General"
color_list = {}
# 初始化HSV值
color_hsv  = {"red"   : ((0, 43, 46), (10, 255, 255)),
              "green" : ((35, 43, 46), (77, 255, 255)),
              "blue"  : ((100, 43, 46), (124, 255, 255)),
              "yellow": ((21, 43, 46), (28, 255, 255))}
HSV_path="/home/jetson/dofbot_pro/dofbot_color_identify/scripts/HSV_config.txt"
# XYT参数路径
XYT_path="/home/jetson/dofbot_pro/dofbot_color_sorting/scripts/XYT_config.txt"
try: read_HSV(HSV_path,color_hsv)
except Exception: print("Read HSV_config Error !!!")
try: xy, threshold = read_XYT(XYT_path)
except Exception: print("Read XYT_config Error !!!")

import Arm_Lib
# 创建机械臂驱动实例
import time
Arm = Arm_Lib.Arm_Device()
joints_0 = [xy[0], xy[1], 0, 0, 90, 30]
Arm.Arm_serial_servo_write6_array(joints_0, 1000)
fps = FPS()

color_list_one = ['red', 'green', 'blue', 'yellow', 'none']




# 抓取控制
def target_detection_Callback(value):
    global model
    model = 'Detection'
    with output: print(model)
def reset_color_list_Callback(value):
    global model
    model = 'Reset_list'
    with output: print(model)
def grap_Callback(value):
    global model
    model = 'Grap'
    with output: print(model)
def exit_button_Callback(value):
    global model
    model = 'Exit'
    with output: print(model)


def camera():
    global color_hsv,model,dp,msg,color_list,color_read
    # 打开摄像头
    capture = cv.VideoCapture(0, cv.CAP_V4L2)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # 当摄像头正常打开的情况下循环执行
    dp = []
    while capture.isOpened():
        try:
            result =  mySpeech.speech_read()
            time.sleep(0.01)
            # 读取相机的每一帧
            _, img = capture.read()
            fps.update_fps()
            if model == 'Calibration':
                time.sleep(0.5)
                dp, img = calibration.calibration_map(img, xy, threshold)
                if len(dp) != 0:
                    model="Detection2"
                    continue
            if len(dp) != 0: img = calibration.Perspective_transform(dp, img)
            if model == 'calibration_Cancel':  
                dp = []
                msg= {}
                color_read = 0
                model="General"
            if model == 'Detection2':
                color_read = 0
                color_list['1'] = "red"
                color_list['2'] = "green"
                color_list['3'] = "blue"
                color_list['4'] = "yellow"
                img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv, color_list)
                time.sleep(0.1)
                mySpeech.void_write(88) 
                time.sleep(2.0)
                print("-+-+-+-+-+")
                if coo1 == 'red':
                    color_read += 1
                    mySpeech.void_write(46)
                    time.sleep(2.0)

                if coo2 == 'green':
                    color_read += 1
                    mySpeech.void_write(49)   
                    time.sleep(2.0)

                if coo3 == 'blue':
                    color_read += 1
                    mySpeech.void_write(48) 
                    time.sleep(2.0)

                if coo4 == 'yellow':
                    color_read += 1
                    mySpeech.void_write(47) 
                    time.sleep(2.0)
                mySpeech.void_write(89)  
                time.sleep(2.0)
                model="list1"
            if model == 'list1' and color_read > 0:  
                if result == 87:
                    color_list['1'] = "red" 
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist1"
                elif result == 88:
                    color_list['1'] = "green"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist1"
                elif result == 89:
                    color_list['1'] = "blue"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist1"
                elif result == 90:
                    color_list['1'] = "yellow"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist1"
                elif result == 91:
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist1"
            if model == 'tolist1':
                if color_read > 0:
                    mySpeech.void_write(84) 
                    model="list2"
                elif color_read == 0:
                    mySpeech.void_write(92)
                    model="toGrap"
            if model == 'list2' and color_read > 0:  
                if result == 87:
                    color_list['2'] = "red" 
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist2"
                elif result == 88:
                    color_list['2'] = "green"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist2"
                elif result == 89:
                    color_list['2'] = "blue"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist2"
                elif result == 90:
                    color_list['2'] = "yellow"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist2"
                elif result == 91:
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist2"
            if model == 'tolist2':
                if color_read > 0:
                    mySpeech.void_write(85)
                    model="list3"
                elif color_read == 0:
                    mySpeech.void_write(92) 
                    model="toGrap"
            if model == 'list3' and color_read > 0:  
                if result == 87:
                    color_list['3'] = "red" 
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist3"
                elif result == 88:
                    color_list['3'] = "green"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist3"
                elif result == 89:
                    color_list['3'] = "blue"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist3"
                elif result == 90:
                    color_list['3'] = "yellow"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist3"
                elif result == 91:
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist3"
            if model == 'tolist3':
                if color_read > 0:
                    mySpeech.void_write(86)
                    model="list4"
                elif color_read == 0:
                    mySpeech.void_write(92)
                    model="toGrap"
            if model == 'list4' and color_read > 0:  
                if result == 87:
                    color_list['4'] = "red" 
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist4"
                elif result == 88:
                    color_list['4'] = "green"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist4"
                elif result == 89:
                    color_list['4'] = "blue"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist4"
                elif result == 90:
                    color_list['4'] = "yellow"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist4"
                elif result == 91:
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist4"
            if model == 'tolist4':
                mySpeech.void_write(92)
                time.sleep(2.0)
                model="toGrap"
            if model == 'toGrap':  
                img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                #coo = {}
                #img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                if result == 92:
                    result = 0
                    model="Grap"

            if result == 109:
                result = 0
                msg={}
                color_list = {}
                model="Detection2"
                mySpeech.void_write(45)
                time.sleep(2.0)
                
                
            if model=="Reset_list":
                #print(coo1)
                #print(coo2)
                #print(coo3)
                #print(coo4)
                msg={}
                color_list = {}
                model="General"
            if len(msg)!= 0 and model == 'Grap':
                threading.Thread(target=target.target_run, args=(msg,xy)).start()
                msg={}
                model="General"
            if model == 'Exit':
                cv.destroyAllWindows()
                capture.release()
                break
            cv.imshow("img", img)
            key = cv.waitKey(1)
        except KeyboardInterrupt:capture.release()                

#第一步先自动标定
model = 'Calibration'
threading.Thread(target=camera, ).start()

