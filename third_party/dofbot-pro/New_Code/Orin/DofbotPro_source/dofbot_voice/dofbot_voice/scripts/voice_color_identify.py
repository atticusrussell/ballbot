import time
import smbus
import Arm_Lib
from Speech_Lib import Speech
import os
from dofbot_utils.robot_controller import Robot_Controller
robot = Robot_Controller()
robot.move_init_pose()
os.system("mpg123 /home/jetson/speech_music/0.mp3")
from dofbot_utils.fps import FPS
fps = FPS()
mySpeech = Speech()
import cv2
import numpy as np

import threading
import inspect
import ctypes

color_ = 0
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(3, 640)
cap.set(4, 480)
cap.set(5, 30)  #设置帧率
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
# image.set(cv2.CAP_PROP_BRIGHTNESS, 40) #设置亮度 -64 - 64  0.0
# image.set(cv2.CAP_PROP_CONTRAST, 50)   #设置对比度 -64 - 64  2.0
# image.set(cv2.CAP_PROP_EXPOSURE, 156)  #设置曝光值 1.0 - 5000  156.0


# 默认选择红色的,程序会自动根据方框中检测到的颜色切换颜色
# 红色区间
color_lower = np.array([0, 43, 46])
color_upper = np.array([10, 255, 255])

def Color_Recongnize():
    while(1):
        #print("---------------------------")
        global color_
        # get a frame and show 获取视频帧并转成HSV格式, 利用cvtColor()将BGR格式转成HSV格式，参数为cv2.COLOR_BGR2HSV。
        ret, frame = cap.read()
        frame, color_name = get_color(frame)
        result = mySpeech.speech_read()
        time.sleep(0.01)
        if len(color_name)==1:
            print("get color!")
            if result == 60:
                if color_name['name'] == 'yellow':
                    mySpeech.void_write(64)
                elif color_name['name'] == 'red':
                    mySpeech.void_write(61)
                elif  color_name['name'] == 'green':
                    mySpeech.void_write(63)
                elif color_name['name'] == 'blue':
                    mySpeech.void_write(62)  
            
        cv2.imshow("res_image", frame)
        key = cv2.waitKey(1)
        time.sleep(0.01)

#线程相关函数
def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # 如果res的值大于1，则需要异常处理
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        
def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


def get_color(img):
    H = []
    color_name={}
    img = cv2.resize(img, (640, 480), )
    # 将彩色图转成HSV
    HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 画矩形框
    cv2.rectangle(img, (280, 180), (360, 260), (0, 255, 0), 2)
    # 依次取出每行每列的H,S,V值放入容器中
    for i in range(280, 360):
        for j in range(180, 260): H.append(HSV[j, i][0])
    # 分别计算出H,S,V的最大最小
    H_min = min(H);H_max = max(H)
    # print(H_min,H_max)
    # 判断颜色
    if H_min >= 0 and H_max <= 10 or H_min >= 156 and H_max <= 180: color_name['name'] = 'red'
    elif H_min >= 21 and H_max <= 28: color_name['name'] = 'yellow'
    elif H_min >= 35 and H_max <= 78: color_name['name'] = 'green'
    elif H_min >= 100 and H_max <= 124: color_name['name'] = 'blue'
    return img, color_name


#启动进程
thread1 = threading.Thread(target=Color_Recongnize)
thread1.setDaemon(True)
thread1.start()

#等待结束进程
try:
    while True:
        time.sleep(0.5)
except KeyboardInterrupt:
    print(" Program closed! ")
    pass
