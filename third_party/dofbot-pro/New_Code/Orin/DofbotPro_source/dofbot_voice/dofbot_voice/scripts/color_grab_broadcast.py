import numpy as np
from Speech_Lib import Speech
import time
from Arm_Lib import Arm_Device
import threading

# 获取机械臂的对象
Arm = Arm_Device()
time.sleep(.1)
import cv2
from dofbot_utils.robot_controller import Robot_Controller
robot = Robot_Controller()
mySpeech = Speech()

# 定义不同位置的变量参数
look_at = robot.P_LOOK_AT
p_top = robot.P_TOP

p_Yellow = robot.P_YELLOW
p_Red = robot.P_RED

p_Green = robot.P_GREEN
p_Blue = robot.P_BLUE

p_gray = robot.P_CENTER

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
#     print(H_min,H_max)
    # 判断颜色
    if H_min >= 0 and H_max <= 20 or H_min >= 156 and H_max <= 180: color_name['name'] = 'red'
    elif H_min >= 21 and H_max <= 28: color_name['name'] = 'yellow'
    elif H_min >= 35 and H_max <= 78: color_name['name'] = 'green'
    elif H_min >= 100 and H_max <= 124: color_name['name'] = 'blue'
    return img, color_name

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
    if enable == 0:
        Arm.Arm_serial_servo_write(6, 60, 400)
    else:
        Arm.Arm_serial_servo_write(6, 135, 400)
    time.sleep(.5)
arm_move_6(look_at, 1000)
time.sleep(1)

global g_state_arm
g_state_arm = 0

def ctrl_arm_move(index):
    arm_clamp_block(0)
    if index == 1:
        print("黄色")
        mySpeech.void_write(66)  
        time.sleep(0.1)
        Arm.Arm_Buzzer_On(1)
        time.sleep(.5)
        number_action(index)
        put_down_block()
        mySpeech.void_write(65) 
        time.sleep(0.1)
    elif index == 2:
        print("红色")
        mySpeech.void_write(69)  
        time.sleep(0.1) 
        Arm.Arm_Buzzer_On(1)
        time.sleep(.5)
        number_action(index)
        put_down_block()
        mySpeech.void_write(65) 
        time.sleep(0.1)  
    elif index == 3:
        print("绿色")
        mySpeech.void_write(67)  
        time.sleep(0.1)  
        Arm.Arm_Buzzer_On(1)
        time.sleep(.5)
        number_action(index)
        put_down_block()
        mySpeech.void_write(65) 
        time.sleep(0.1)  
    elif index == 4:
        print("蓝色")
        mySpeech.void_write(68)  
        time.sleep(0.1)  
        Arm.Arm_Buzzer_On(1)
        time.sleep(.5)
        number_action(index)
        put_down_block()
        mySpeech.void_write(65) 
        time.sleep(0.1)
    
    global g_state_arm
    g_state_arm = 0

#数字功能定义
def number_action(index):
    if index == 1:
        # 抓取黄色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Yellow, 1500)
    elif index == 2:
        # 抓取红色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Red, 1500)
    elif index == 3:
        # 抓取绿色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Green, 1500)
    elif index == 4:
        # 抓取蓝色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Blue, 1500)
    arm_clamp_block(1)
    arm_move(p_top, 1500)

    
def put_down_block():
    arm_move(p_gray, 1000)
    arm_clamp_block(0) 
    time.sleep(.5)
    arm_move_6(look_at, 1500)

def start_move_arm(index):
    # 开启机械臂控制线程
    global g_state_arm
    if g_state_arm == 0:
        closeTid = threading.Thread(target = ctrl_arm_move, args = [index])
        closeTid.setDaemon(True)
        closeTid.start()
        
        g_state_arm = 1


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
    
    while(1):
        # get a frame and show 获取视频帧并转成HSV格式, 利用cvtColor()将BGR格式转成HSV格式，参数为cv2.COLOR_BGR2HSV。
        ret, frame = cap.read()
        frame, color_name = get_color(frame)
        if len(color_name)==1:
            if color_name['name'] == 'yellow':
                start_move_arm(1)
            elif color_name['name'] == 'red':
                start_move_arm(2)
            elif  color_name['name'] == 'green':
                start_move_arm(3) 
            elif color_name['name'] == 'blue':
                start_move_arm(4)
        


        cv2.imshow("res_image", frame)
        key = cv2.waitKey(1)
        time.sleep(0.01)


#启动进程
thread1 = threading.Thread(target=Color_Recongnize)
thread1.setDaemon(True)
thread1.start()

#等待结束进程
try:
    while True:
        time.sleep(.000001)
except KeyboardInterrupt:
    print(" Program closed! ")
    pass


