import time
import smbus
from Speech_Lib import Speech
from Arm_Lib import Arm_Device
import os
from dofbot_utils.robot_controller import Robot_Controller
robot = Robot_Controller()
Arm = Arm_Device()
os.system("mpg123 /home/jetson/speech_music/0.mp3")
mySpeech = Speech()
import cv2
import numpy as np
import threading
import inspect
import ctypes


# 定义移动机械臂函数,同时控制1-6号舵机运动，p=[S1,S2,S3,S4,S5,S6]
def arm_move_6(p, s_time = 500):
    for i in range(6):
        id = i + 1
        Arm.Arm_serial_servo_write(id, p[i], s_time)
        time.sleep(.01)
    time.sleep(s_time/1000)
    
# 定义移动机械臂函数,同时控制1-5号舵机运动，p=[S1,S2,S3,S4,S5]
def arm_move(p, s_time = 500):
    for i in range(5):
        id = i + 1
        if id == 5:
            time.sleep(.1)
            Arm.Arm_serial_servo_write(id, p[i], int(s_time*1.2))
        elif id == 1 :
            Arm.Arm_serial_servo_write(id, p[i], int(3*s_time/4))
        else:
            Arm.Arm_serial_servo_write(id, p[i], int(s_time))
        time.sleep(.01)
    time.sleep(s_time/1000)
    
# 定义夹积木块函数，enable=1：夹住，=0：松开
def arm_clamp_block(enable):
    robot.arm_clamp_block(enable, 400)
    time.sleep(.5)

# 定义不同位置的变量参数
look_at = robot.P_LOOK_AT
p_top = robot.P_TOP

p_Yellow = robot.P_YELLOW
p_Red = robot.P_RED

p_Green = robot.P_GREEN
p_Blue = robot.P_BLUE

p_layer_4 = robot.P_CENTER_HEAP_L4
p_layer_3 = robot.P_CENTER_HEAP_L3
p_layer_2 = robot.P_CENTER_HEAP_L2
p_layer_1 = robot.P_CENTER_HEAP_L1


p_push_over_1 = robot.P_OVER_1
p_push_over_2 = robot.P_OVER_2


#定义抓取方块的状态
yellow_grabbed = 0
red_grabbed = 0
green_grabbed = 0
blue_grabbed = 0

arm_move_6(look_at, 1000)
time.sleep(1)

#数字功能定义
def number_action(index):
    if index == 1:
        # 抓取黄色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Yellow, 1000)
        arm_clamp_block(1)
        Arm.Arm_serial_servo_write(2, 90, 1000)
        time.sleep(1)
        arm_move(p_top, 1000)
    elif index == 2:
        # 抓取红色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Red, 1000)
        arm_clamp_block(1)
        Arm.Arm_serial_servo_write(2, 90, 1000)
        time.sleep(1)
        arm_move(p_top, 1000)
    elif index == 3:
        # 抓取绿色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Green, 1000)
        arm_clamp_block(1)
        Arm.Arm_serial_servo_write(2, 90, 1000)
        time.sleep(1)
        arm_move(p_top, 1000)
    elif index == 4:
        # 抓取蓝色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Blue, 1000)
        arm_clamp_block(1)
        Arm.Arm_serial_servo_write(2, 90, 1000)
        time.sleep(1)
        arm_move(p_top, 1000)

def put_down_block(layer):
    
    if layer == 1:
        arm_move(p_layer_1, 1000)
        arm_clamp_block(0) 
        arm_move_6(look_at, 1000)
    elif layer == 2:
        arm_move(p_layer_2, 1000)
        arm_clamp_block(0)
        arm_move_6(look_at, 1000)
    elif layer == 3:
        arm_move(p_layer_3, 1000)
        arm_clamp_block(0) 
        arm_move_6(look_at, 1000)
    elif layer == 4:
        arm_move(p_layer_4, 1000)
        time.sleep(.1)
        arm_clamp_block(0) 
        arm_move_6(look_at, 1000)
    mySpeech.void_write(81)  
    
# 推倒积木块
def push_over_block():
    arm_move_6(p_push_over_1, 1000)
    time.sleep(.2)
    arm_move_6(p_push_over_2, 1000)
    time.sleep(.1)
    arm_move_6(look_at, 1000)
    time.sleep(1)
    global g_layer
    g_layer = 0

global g_state_arm
g_state_arm = 0

global g_layer
g_layer = 0

def ctrl_arm_move(index):
    global g_layer
    g_layer = g_layer + 1
    if g_layer >= 5:
        g_layer = 1
    arm_clamp_block(0)
    if index == 1:
        number_action(index)
        time.sleep(1)
        put_down_block(g_layer)
    elif index == 2:
        number_action(index)
        time.sleep(1)
        put_down_block(g_layer)
    elif index == 3:
        number_action(index)
        time.sleep(1)
        put_down_block(g_layer)
    elif index == 4:
        number_action(index)
        time.sleep(1)
        put_down_block(g_layer)
    elif index == 5:
        time.sleep(1)
        push_over_block()
    
        
    global g_state_arm
    g_state_arm = 0

def start_move_arm(index):
    # 开启机械臂控制线程
    global g_state_arm
    if g_state_arm == 0:
        closeTid = threading.Thread(target = ctrl_arm_move, args = [index])
        closeTid.setDaemon(True)
        closeTid.start()
        
        g_state_arm = 1

try:   
    Arm.Arm_Buzzer_On(1)
    s_time = 300
    Arm.Arm_serial_servo_write(4, 10, s_time)
    time.sleep(s_time/1000)
    Arm.Arm_serial_servo_write(4, 0, s_time)
    time.sleep(s_time/1000)
    Arm.Arm_serial_servo_write(4, 10, s_time)
    time.sleep(s_time/1000)
    Arm.Arm_serial_servo_write(4, 0, s_time)
    time.sleep(s_time/1000)
    while True:
        res = mySpeech.speech_read()
        time.sleep(0.01)
        if res == 77:
            if yellow_grabbed == 0:
                mySpeech.void_write(77)  
                Arm.Arm_Buzzer_On(1)
                time.sleep(.1)
                start_move_arm(1)
                time.sleep(1)
                # global yellow_grabbed
                yellow_grabbed = 1
        elif res == 78:
            if red_grabbed == 0:
                mySpeech.void_write(77)   
                Arm.Arm_Buzzer_On(1)
                time.sleep(.1)
                start_move_arm(2)
                # global red_grabbed
                red_grabbed = 1
        elif res == 79:
            if green_grabbed == 0:
                mySpeech.void_write(77)   
                Arm.Arm_Buzzer_On(1)
                time.sleep(.1)
                start_move_arm(3)
                # global green_grabbed
                green_grabbed = 1
        elif res == 80:
            if blue_grabbed == 0:
                mySpeech.void_write(77) 
                Arm.Arm_Buzzer_On(1)
                time.sleep(.1)
                start_move_arm(4)
                # global blue_grabbed
                blue_grabbed = 1
        elif res == 82: #推倒
            mySpeech.void_write(82) 
            Arm.Arm_Buzzer_On(1)
            time.sleep(.1)
            start_move_arm(5)
            yellow_grabbed = 0
            red_grabbed = 0
            green_grabbed = 0
            blue_grabbed = 0

except KeyboardInterrupt:
    print(" Program closed! ")
    pass


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
            global color_lower
            global color_upper
            # print ("color_name :", color_name)
            # print ("name :", color_name['name'])
            if color_name['name'] == 'yellow':
                color_lower = np.array([21, 43, 46])
                color_upper = np.array([28, 255, 255])
                color_ = 1
            elif color_name['name'] == 'red':
                color_lower = np.array([0, 43, 46])
                color_upper = np.array([10, 255, 255])
                color_ = 2
            elif  color_name['name'] == 'green':
                color_lower = np.array([35, 43, 46])
                color_upper = np.array([77, 255, 255])
                color_ = 3
            elif color_name['name'] == 'blue':
                color_lower=np.array([100, 43, 46])
                color_upper = np.array([124, 255, 255])
                color_ = 4
        
        if result == 60:
            #yellow
            if color_ == 1:
                mySpeech.void_write(64)#等待当前语句播报结束
                time.sleep(0.1)  
            elif color_ == 2:
                mySpeech.void_write(61)#等待当前语句播报结束
                time.sleep(0.1) 
            elif color_ == 3:
                mySpeech.void_write(63)#等待当前语句播报结束
                time.sleep(0.1) 
            elif color_ == 4:
                mySpeech.void_write(62)#等待当前语句播报结束
                time.sleep(0.1)   
            
        #origin_widget.value = bgr8_to_jpeg(frame)
        # change to hsv model
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # get mask 利用inRange()函数和HSV模型中蓝色范围的上下界获取mask，mask中原视频中的蓝色部分会被弄成白色，其他部分黑色。
        mask = cv2.inRange(hsv, color_lower, color_upper)
        #cv2.imshow('Mask', mask)
        #mask_widget.value = bgr8_to_jpeg(mask)

        # detect blue 将mask于原视频帧进行按位与操作，则会把mask中的白色用真实的图像替换：
        res = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("res_image", frame)
        key = cv2.waitKey(1)
        #cv2.imshow('Result', res)
        #result_widget.value = bgr8_to_jpeg(res)
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
