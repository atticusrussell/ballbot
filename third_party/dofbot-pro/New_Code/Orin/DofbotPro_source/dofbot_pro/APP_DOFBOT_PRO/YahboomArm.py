#coding=utf-8
from flask import Flask, render_template, Response
from camera import VideoCamera
import socket
import os
import random
import time
from Arm_Lib import Arm_Device
import threading
import cv2
from PIL import Image,ImageDraw,ImageFont
import numpy as np
import sys
import rclpy
from rclpy.node import Node
import configparser  #配置文件

sys.path.append("/home/jetson/colcon_ws/src/dofbot_color_identify/scripts")
sys.path.append("/home/jetson/colcon_ws/src/dofbot_color_follow")
sys.path.append("/home/jetson/colcon_ws/src/dofbot_face_follow/scripts")


sys.path.append("/home/jetson/colcon_ws/src/dofbot_color_stacking/scripts")
sys.path.append("/home/jetson/colcon_ws/src/dofbot_garbage_yolov11")

sys.path.append("/home/jetson/colcon_ws/src/dofbot_snake_follow/scripts")

sys.path.append("/home/jetson/colcon_ws/src/dofbot_color_sorting/scripts")

sys.path.append("/home/jetson/colcon_ws/src/dofbot_color_grab/scripts")
sys.path.append("/home/jetson/colcon_ws/src/dofbot_action_group")
sys.path.append("/home/jetson/colcon_ws/src/dofbot_gesture")

sys.path.append("/opt/ros/humble/bin/")
sys.path.append("/opt/ros/humble/lib/python3.10/site-packages")

#ROS
def waitRos():
    os.system("bash /home/jetson/dofbot_pro_ws/install/setup.bash")
    cmd = "ros2 run dofbot_pro_info kinemarics_dofbot"
    os.system(cmd)
    
rosTid = threading.Thread(target = waitRos, args = [])
rosTid.daemon = True  
rosTid.start()

#标定
from Calibration import Arm_Calibration
calibration = Arm_Calibration()

#颜色校准
from Calibration import update_hsv
colorCalibration = update_hsv()

rclpy.init(args=None)
#颜色跟踪
from color_follow_ctrl import color_follow
Arm_color_follow = color_follow()

#颜色堆叠
from color_stacking import color_stacking
Arm_stack = color_stacking()

#颜色分拣
from color_identify import color_identify
Arm_identify = color_identify()

# 垃圾分拣
from garbage_identify import garbage_identify
Arm_garbage = garbage_identify()

# 单个垃圾分拣
from single_garbage_identify import single_garbage_identify
Arm_single_garbage = single_garbage_identify()

#人脸追踪
from face_follow import Face_Follow
Arm_face_follow = Face_Follow()

#你放我抓
from color_sorting import color_sorting
Arm_sorting = color_sorting()

#引蛇出洞
from snake_target import snake_target
from snake_ctrl import snake_ctrl
Arm_snake_target = snake_target()
Arm_snake_ctrl = snake_ctrl()

#ayue 
#手势识别动作
from gesture_action import gesture_action
Arm_gesture_action = gesture_action()

#手势堆叠
from gesture_stack import gesture_stack
Arm_gesture_stack = gesture_stack()

#颜色抓取
from color_grab import Color_Grab
Arm_color_grab = Color_Grab()

#自定义动作组
from action_group import action_group
Arm_action_group = action_group()

#wifi配网
#from Arm_WIFI import Arm_WIFI
#Arm_wifi = Arm_WIFI()
#ayue

global g_init
g_init = False
Arm = Arm_Device()
global g_mode
g_mode = 'Standard'
global g_calibrateMode
g_calibrateMode = 'None'
global g_calibrateXY


global g_colorstudyMode
g_colorstudyMode = 'None'

global g_colororder
g_colororder = '00000'

global g_colorDisplay
g_colorDisplay = 'Normal'

global g_wifi
g_wifi = False

app = Flask(__name__)


#读取配置文件####################################################################################
g_calibrateThreshold = 140

# g_color_dict = {'red': ((3, 39, 81), (19, 255, 255)),
#                 'green': ((47, 73, 73), (76, 255, 255)),
#                 'blue': ((109, 133, 139), (121, 255, 255)),
#                 'yellow': ((20, 23, 199), (30, 255, 255))}
g_color_dict =  {"red": ((0, 150, 160), (10, 255, 255)),
                  "green": ((35, 80, 36), (77, 255, 255)),
                  "blue": ((110, 124, 100), (124, 253, 255)),
                  "yellow": ((26, 98, 130), (34, 255, 255))}

g_color_dict_temp =  {"red": ((0, 150, 160), (10, 255, 255)),
                  "green": ((35, 80, 36), (77, 255, 255)),
                  "blue": ((110, 124, 100), (124, 253, 255)),
                  "yellow": ((26, 98, 130), (34, 255, 255))}

g_HSV_Red = [0, 150, 160, 10, 255, 255] 
g_HSV_Green = [53, 80, 36, 80, 255, 255] 
g_HSV_Blue = [110, 124, 100, 124, 253, 255] 
g_HSV_Yellow = [20, 98, 130, 40, 255, 255] 
g_calibrateXY = [90, 135]

#base_dir = str(os.getcwd())
#base_dir = base_dir.replace('\\', '/')
#file_path = base_dir + "/config.ini"
file_path = "/home/jetson/dofbot_pro/APP_DOFBOT_PRO/config.ini"
 
cf = configparser.ConfigParser()   # configparser类来读取config文件

try:
    with open(file_path,mode='r') as ff:
        print(ff.readlines())
except FileNotFoundError:
    cf.add_section("calibrateThreshold")
    cf.add_section("HSV")
    cf.add_section("calibrateXY")
    cf.set('HSV', 'g_HSV_Red', str(", ".join(repr(e) for e in g_color_dict["red"])).replace('(', '').replace(')', ''))
    cf.set('HSV', 'g_HSV_Green', str(", ".join(repr(e) for e in g_color_dict["green"])).replace('(', '').replace(')', ''))
    cf.set('HSV', 'g_HSV_Blue', str(", ".join(repr(e) for e in g_color_dict["blue"])).replace('(', '').replace(')', ''))
    cf.set('HSV', 'g_HSV_Yellow', str(", ".join(repr(e) for e in g_color_dict["yellow"])).replace('(', '').replace(')', ''))
    cf.set('calibrateThreshold', 'g_calibratethreshold', str(g_calibrateThreshold))
    cf.set('calibrateXY', 'g_calibrateXY', str(", ".join(repr(e) for e in g_calibrateXY)))
    with open(file_path, 'w')as conf:
        cf.write(conf)
        print("配置文件创建成功！")

cf.read(file_path)

#颜色标定阈值
try:
    g_calibrateThreshold = int(cf.get("calibrateThreshold", "g_calibrateThreshold"))
    print(g_calibrateThreshold)
except:
    print("No g_calibrateThreshold moren:")
    g_calibrateThreshold = 140
    print(g_calibrateThreshold)

#标定机械臂位置
try:
    data1 = cf['calibrateXY']['g_calibrateXY']
    g_calibrateXY = data1.split(',')
    g_calibrateXY = list(map(int, g_calibrateXY))
    print(g_calibrateXY)
except:
    g_calibrateXY = [90, 135]
    print("No data1 moren:")
    print(g_calibrateXY)

#颜色HSV值    
try:    
    data2 = cf['HSV']['g_HSV_Red']
    g_HSV_Red = data2.split(',')
    g_HSV_Red = list(map(int, g_HSV_Red))
    g_color_dict["red"] = ((g_HSV_Red[0], g_HSV_Red[1] , g_HSV_Red[2] ), (g_HSV_Red[3], g_HSV_Red[4], g_HSV_Red[5]))
    print(g_HSV_Red)
except:
    g_HSV_Red = [0, 150, 160, 10, 255, 255]
    print("No data2 moren")
    print(g_HSV_Red)
    
try:    
    data3 = cf['HSV']['g_HSV_Green']
    g_HSV_Green = data3.split(',')
    g_HSV_Green = list(map(int, g_HSV_Green))
    g_color_dict["green"] = ((g_HSV_Green[0], g_HSV_Green[1] , g_HSV_Green[2] ), (g_HSV_Green[3], g_HSV_Green[4], g_HSV_Green[5]))
    print(g_HSV_Green)
except:
    g_HSV_Green = [53, 80, 36, 80, 255, 160]
    print("No data3 moren")
    print(g_HSV_Green)

try:    
    data4 = cf['HSV']['g_HSV_Blue']
    g_HSV_Blue = data4.split(',')
    g_HSV_Blue = list(map(int, g_HSV_Blue))
    g_color_dict["blue"] = ((g_HSV_Blue[0], g_HSV_Blue[1] , g_HSV_Blue[2] ), (g_HSV_Blue[3], g_HSV_Blue[4], g_HSV_Blue[5]))
    print(g_HSV_Blue)
except:
    g_HSV_Blue = [110, 124, 100, 124, 253, 255]
    print("No data4 moren")
    print(g_HSV_Blue)
    
try:    
    data5 = cf['HSV']['g_HSV_Yellow']
    g_HSV_Yellow = data5.split(',')
    g_HSV_Yellow = list(map(int, g_HSV_Yellow))
    g_color_dict["yellow"] = ((g_HSV_Yellow[0], g_HSV_Yellow[1] , g_HSV_Yellow[2] ), (g_HSV_Yellow[3], g_HSV_Yellow[4], g_HSV_Yellow[5]))
    print(g_HSV_Yellow)
except:
    g_HSV_Yellow = [53, 80, 36, 80, 255, 160]
    print("No data5 moren")
    print(g_HSV_Yellow)

######################## 读取配置文件结束 ########################################################


g_color_hsv = None
# 看地图中心初始位置
joints_init_down = [90, 125, 0, 0, 90, 0]
# 向前看
joints_init_front = [90, 135, 0,45, 90, 0]
# 追踪的初始位置
joints_follow = [90, 135, 20, 25, 90, 0]


    
# 获取本机IP
def getLocalip():
    ip = os.popen("/sbin/ifconfig eno1 | grep 'inet' | awk '{print $2}'").read()
    ip = ip[0 : ip.find('\n')]
    if(ip == ''):
        #读取WLAN的IP
        ip = os.popen("/sbin/ifconfig wlP1p1s0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[0 : ip.find('\n')]
        if(ip == ''):
            ip = 'x.x.x.x'
    return ip

def waitMidSetAll(socket):
    for i in range(1,7):
        print(type(i))
        id = int(i)
        Arm.Arm_serial_servo_write_offset_switch(id)
        time.sleep(0.1)
        state = Arm.Arm_serial_servo_write_offset_state()
        checksum = (0x0A + 4 + id + int(state))&0xff
        idstr = "%02d" % id
        statestr = "%02d" % state
        checksumstr = "%02x" % checksum
        data = "$0A04" + idstr + statestr+ checksumstr + "#"
        socket.send(data.encode(encoding="utf-8")) #返回舵机中值设置状态
        print(data)
        
#自定义动作组进程
def waitCustomActionGroup(index):
    Arm_action_group.start_action(index)

#协议解析部分
def Analysis(socket, str):
    import time
    global g_colororder, g_mode, g_calibrateMode, g_colorstudyMode, g_calibrateXY
    global g_color_hsv, g_calibrateThreshold, g_colorDisplay
    print(str)
    #    $04070113795#
    cmd = str[1:3]
    if cmd == "00":
        page = int(str[5:7])
        if page == 0:
            g_mode = 'Standard'
            g_calibrateMode = 'None'
            g_colorstudyMode = 'None'
            Arm.Arm_RGB_set(0, 0, 0)
        elif page == 1: # 动作组
            Arm.Arm_serial_servo_write6_array(joints_init_down, 1000)
        elif page == 2: # 手势
            Arm.Arm_serial_servo_write6_array(joints_init_front, 1000)
        elif page == 3: # 颜色互动
            Arm.Arm_serial_servo_write6_array(joints_init_down, 1000)
        elif page == 4: # 追踪
            Arm.Arm_serial_servo_write6_array(joints_follow, 1000)
        elif page == 5: # 单个垃圾分拣
            Arm.Arm_serial_servo_write6_array(joints_init_down, 1000)
        
    elif cmd == "01": #获取硬件版本号
        data = "$0102100d#"
        socket.send(data.encode(encoding="utf-8"))
    elif cmd == "02": #获取指定舵机的位置
        id = str[5:7]
        pos = Arm.Arm_serial_servo_read(id)
        checksum = (2 + 5 + id + pos)&0xff
        checksumstr = "%02x" % checksum
        posstr = "%02d" % pos
        data = "$0205" + str[5:7] + posstr + checksumstr + "#"
        socket.send(data.encode(encoding="utf-8"))
    elif cmd == "03": #获取前几个舵机的位置
#         pos = [0,0,0,0,0,0]
#         for i in range(0, 6):
#             pos[i] = Arm.Arm_serial_servo_read(i+1)
#             if pos[i] == None:
#                 return
#             time.sleep(0.001)
        pos = [0,0,0,0,0,0]
        for i in range(0, 6):
            num = 0
            while num<10:
                # 读取舵机角度
                joint = Arm.Arm_serial_servo_read(i+1)
                time.sleep(0.01)
                # 每当读取到数据时,跳出循环,返回结果
                if joint != None:
                    pos[i] = joint
                    break
                num += 1
        checksum = (3 + 20 + 6 + pos[0] + pos[1] + pos[2] + pos[3] + pos[4] + pos[5])&0xff
        pos1str = "%03d" % pos[0]
        pos2str = "%03d" % pos[1]
        pos3str = "%03d" % pos[2]
        pos4str = "%03d" % pos[3]
        pos5str = "%03d" % pos[4]
        pos6str = "%03d" % pos[5]
        checksumstr = "%02x" % checksum
        data = "$0320" + str[5:7] + pos1str+pos2str+pos3str+pos4str+pos5str+pos6str + checksumstr + "#"
        print (data)
        socket.send(data.encode(encoding="utf-8"))
    elif cmd == "04":  #设置指定舵机位置
        id = int(str[5:7])
        angle = int(str[7:10])
        Arm.Arm_serial_servo_write(id, angle, 500)
    elif cmd == "05":  #设置一组舵机位置
        #    $0524060001800001800001809999XX#
        idmax = int(str[5:7])
        if idmax == 6:
            angle1 = int(str[7:10])
            angle2 = int(str[10:13])
            angle3 = int(str[13:16])
            angle4 = int(str[16:19])
            angle5 = int(str[19:22])
            angle6 = int(str[22:25])
            time = int(str[25:29])
            Arm.Arm_serial_servo_write6(angle1, angle2, angle3, angle4, angle5, angle6, time)
    elif cmd == "06":  #设置舵机ID地址
        id = int(str[5:7])
        if id < 1 or id > 250:
            socket.send("$0602020A#".encode(encoding="utf-8")) #编号错误
            return
        Arm.Arm_serial_set_id(id)    
        socket.send("$06020008#".encode(encoding="utf-8")) #返回ok
    elif cmd == "07":  #归中舵机位置
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 180, 1000)
    elif cmd == "08":  #自定义动作组
        m_index = int(str[5:7])
        action_state = Arm_action_group.read_state()
        if m_index == 0: #关闭动作组
            Arm_action_group.set_state(0)
        elif action_state != 1:
            Arm_action_group.set_state(1)
            closeTid = threading.Thread(target = waitCustomActionGroup, args = [m_index])
            closeTid.daemon = True
            closeTid.start()
        g_mode = 'RubbishSorting ...'

    elif cmd == "09":  #急停|恢复舵机，关闭舵机
        state = int(str[5:6])
        Arm.Arm_serial_set_torque(state)
    elif cmd == "0A":  #设置舵机中值id:0 恢复默认  id：1-20 正常舵机中位值
        if str[5:7] == 'ff': #全部设置
            closeTid = threading.Thread(target = waitMidSetAll, args = [socket])
            closeTid.daemon = True
            closeTid.start()
            return
        id = int(str[5:7])
        print(id)
        #单独设置
        Arm.Arm_serial_servo_write_offset_switch(id)
        time.sleep(0.05)
        state = Arm.Arm_serial_servo_write_offset_state()
        checksum = (0x0A + 4 + id + state)&0xff
        idstr = "%02d" % id
        statestr = "%02d" % state
        checksumstr = "%02x" % checksum
        data = "$0A04" + idstr + statestr+ checksumstr + "#"
        socket.send(data.encode(encoding="utf-8")) #返回舵机中值设置状态
        print(state)
    elif cmd == "0B": #动作组模式选择
        mode = int(str[5:7])
        if mode == 1: #学习模式
            Arm.Arm_Button_Mode(1)
        elif mode == 2: #退出学习模式
            Arm.Arm_Button_Mode(0)
        elif mode == 3: #清空动作组
            Arm.Arm_Clear_Action()
        elif mode == 4: #单次运行动作组
            Arm.Arm_Action_Mode(1)
        elif mode == 5: #循环运行动作组
            Arm.Arm_Action_Mode(2)
        elif mode == 6: #停止动作组
            Arm.Arm_Action_Mode(0)
        elif mode == 8: #停止动作组
            Arm.Arm_Action_Mode(0)
        elif mode == 7: #学习动作组
            Arm.Arm_Action_Study()
        elif mode == 9: #读取已学习的动作组数量
            group_num = Arm.Arm_Read_Action_Num()
            group_numstr = "%02d" % group_num
            checksum = (0x0B + 2 + group_num) & 0xff
            checksumstr = "%02d" % checksum
            data = "$0B02" + group_numstr + checksumstr + "#"
            socket.send(data.encode(encoding="utf-8"))
    elif cmd == "0C": #控制RGB灯
        color = int(str[5:7])
        if color == 0:
            Arm.Arm_RGB_set(0, 0, 0)
        elif color == 1:
            Arm.Arm_RGB_set(255, 0, 0)
        elif color == 2:
            Arm.Arm_RGB_set(0, 255, 0)
        elif color == 3:
            Arm.Arm_RGB_set(0, 0, 255)
        elif color == 4:
            Arm.Arm_RGB_set(255, 255, 0)
    elif cmd == "0D": #颜色分拣
        g_colororder = str[6:10] #暂且传4个
        switch = int(str[5])
        if switch == 1:
            g_mode = 'colorsortstart'
            print("颜色分拣开始")
        elif switch == 0:
            g_mode = 'colorsortplan'
            print("颜色分拣发送颜色顺序")
    elif cmd == "0E": #颜色堆叠
        g_colororder = str[6:10] #暂且传4个
        switch = int(str[5])
        if switch == 1:
            g_mode = 'colorstackstart'
            print("颜色堆叠开始")
        elif switch == 0:
            g_mode = 'colorstackplan'
            print("颜色堆叠发送颜色顺序")
        print("颜色堆叠模式")
    elif cmd == "0F": #人脸追踪   $0F05014#
        switch = int(str[5])
        if switch == 1:
            g_mode = 'facefunction'
            print("人脸追踪模式开")
        elif switch == 0:
            g_mode = 'Standard'
            print("人脸追踪模式关")
    elif cmd == "10": #颜色追踪
        color = int(str[5:7])
        print(color)
        if color == 0:
            g_mode = 'Standard'
            Arm.Arm_RGB_set(0, 0, 0)
        elif color == 1:
            g_mode = 'colortrack1'
            Arm.Arm_RGB_set(255, 0, 0)
        elif color == 2:
            g_mode = 'colortrack2'
            Arm.Arm_RGB_set(0, 255, 0)
        elif color == 3:
            g_mode = 'colortrack3'
            Arm.Arm_RGB_set(0, 0, 255)
        elif color == 4:
            g_mode = 'colortrack4'
            Arm.Arm_RGB_set(255, 255, 0)
        elif color == 5:
            g_mode = 'colortrackstudy'
        elif color == 6:
            g_mode = 'colortrackstudyOK'
        print(g_mode)
        print("颜色追踪模式")   
    elif cmd == "11": #识别区域标定
        mode = int(str[5:7])
        if mode == 1: #进入标定模式
            g_calibrateMode = 'calibrateMode'
            print("进入标定模式")
        elif mode == 2: #标定确定
            g_calibrateMode = 'calibrateOK'
            print("标定确定")
        elif mode == 3: #标定退出
            g_calibrateMode = 'calibrateCancel'
            print("标定退出")
    elif cmd == "12": #垃圾分类
        mode = int(str[5:7])
        if mode == 1: #进入垃圾识别模式
            g_mode = 'rubbishplan'
            print("垃圾识别")
        elif mode == 2: #垃圾分拣
            g_mode = 'rubbishOK'
            print("垃圾分拣")
        elif mode == 3: #进入垃圾识别模式
            g_mode = 'single_rubbishplan'
            print("单个垃圾识别")
        elif mode == 4: #退出垃圾分拣
            g_mode = 'Standard'
            print("退出")


#     elif cmd == "13": #颜色学习
#         mode = int(str[5:6])
#         g_colorstudy = int(str[6:7])
#         if g_colorstudy==1:
#             g_color_hsv="red"
#         elif g_colorstudy==2:
#             g_color_hsv="green"
#         elif g_colorstudy==3:
#             g_color_hsv="blue"
#         elif g_colorstudy==4:
#             g_color_hsv="yellow"
#         print(g_color_hsv)
#         if mode == 1:
#             g_colorstudyMode = 'colorstudyMode'
#         elif mode == 2:
#             g_colorstudyMode = 'colorstudyModeOK'
#         elif mode == 3:
#             g_colorstudyMode = 'colorstudyModeCancel'
    elif cmd == '14': #标定阈值
        g_calibrateThreshold = int(str[5:8])
        print(g_calibrateThreshold)
    elif cmd == '15': #标定位置微调
        mode = int(str[5:6])
        if mode == 1:   #上
            g_calibrateXY[1] = g_calibrateXY[1] + 1
        elif mode == 2: #下
            g_calibrateXY[1] = g_calibrateXY[1] - 1
        elif mode == 3: #左
            g_calibrateXY[0] = g_calibrateXY[0] + 1
        elif mode == 4: #右
            g_calibrateXY[0] = g_calibrateXY[0] - 1
    elif cmd == '16': #手势识别玩法
        mode = int(str[5:6])
        switch = int(str[6:7])
        if mode == 1: #手势识别动作
            if switch == 1:
                g_mode = 'GestureActionMode'
            else:
                Arm_gesture_action.reset_state()
                g_mode = 'Standard'
        if mode == 2: #手势识别堆叠
            if switch == 1:
                g_mode = 'GestureStackMode'
            else:
                Arm_gesture_stack.reset_state()
                g_mode = 'Standard'
    elif cmd == '17': #颜色识别玩法
        mode = int(str[5:6])
        switch = int(str[6:7])
        if mode == 1: #你放我抓
            if switch == 1:
                Arm.Arm_serial_servo_write6_array(joints_init_down, 500)
                g_mode = 'YouputWecatchMode'
            else:
                g_mode = 'Standard'
        elif mode == 2: #引蛇出洞
            Arm.Arm_serial_servo_write6_array(joints_init_front, 1500)
            if switch == 0:
                g_mode = 'Standard'
                #关闭灯
                Arm.Arm_RGB_set(0, 0, 0)
            elif switch == 1:
                g_mode = 'SnakeoutholeMode1'
            elif switch == 2:
                g_mode = 'SnakeoutholeMode2'
            elif switch == 3:
                g_mode = 'SnakeoutholeMode3'
            elif switch == 4:
                g_mode = 'SnakeoutholeMode4'
        elif mode == 3: #颜色抓取
            if switch == 1:
                g_mode = 'ColorRecogFllowCatchMode'
            else:
                Arm_color_grab.reset_state()
                g_mode = 'Standard'
    elif cmd == '18': # 颜色校准界面切换
        mode = int(str[5:6])
        if mode == 1:
            g_colorDisplay = 'ColorCalibration'
        else:
            g_colorDisplay = 'Normal'
        print(g_colorDisplay)
    elif cmd == '19': #向导界面：颜色学习校准
        g_colorstudy = int(str[5:6])
        mode = int(str[6:7])
        if g_colorstudy==1:
            g_color_hsv = "red"
            checksum = (19 + 18 + g_color_dict[g_color_hsv][0][0] +
                        g_color_dict[g_color_hsv][0][1] +
                        g_color_dict[g_color_hsv][0][2] +
                        g_color_dict[g_color_hsv][1][0] +
                        g_color_dict[g_color_hsv][1][1] +
                        g_color_dict[g_color_hsv][1][2]) & 0xff
            H_min = "%03d" % g_color_dict[g_color_hsv][0][0]
            S_min = "%03d" % g_color_dict[g_color_hsv][0][1]
            V_min = "%03d" % g_color_dict[g_color_hsv][0][2]
            H_max = "%03d" % g_color_dict[g_color_hsv][1][0]
            S_max = "%03d" % g_color_dict[g_color_hsv][1][1]
            V_max = "%03d" % g_color_dict[g_color_hsv][1][2]
            checksumstr = "%02x" % checksum
            data = "$1918" + H_min+S_min+V_min+H_max+S_max+V_max + checksumstr + "#"
            g_socket.send(data.encode(encoding="utf-8"))
        elif g_colorstudy==2:
            g_color_hsv="green"
            checksum = (19 + 18 + g_color_dict[g_color_hsv][0][0] +
                        g_color_dict[g_color_hsv][0][1] +
                        g_color_dict[g_color_hsv][0][2] +
                        g_color_dict[g_color_hsv][1][0] +
                        g_color_dict[g_color_hsv][1][1] +
                        g_color_dict[g_color_hsv][1][2]) & 0xff
            H_min = "%03d" % g_color_dict[g_color_hsv][0][0]
            S_min = "%03d" % g_color_dict[g_color_hsv][0][1]
            V_min = "%03d" % g_color_dict[g_color_hsv][0][2]
            H_max = "%03d" % g_color_dict[g_color_hsv][1][0]
            S_max = "%03d" % g_color_dict[g_color_hsv][1][1]
            V_max = "%03d" % g_color_dict[g_color_hsv][1][2]
            checksumstr = "%02x" % checksum
            data = "$1918" + H_min+S_min+V_min+H_max+S_max+V_max + checksumstr + "#"
            g_socket.send(data.encode(encoding="utf-8"))
        elif g_colorstudy==3:
            g_color_hsv="blue"
            checksum = (19 + 18 + g_color_dict[g_color_hsv][0][0] +
                        g_color_dict[g_color_hsv][0][1] +
                        g_color_dict[g_color_hsv][0][2] +
                        g_color_dict[g_color_hsv][1][0] +
                        g_color_dict[g_color_hsv][1][1] +
                        g_color_dict[g_color_hsv][1][2]) & 0xff
            H_min = "%03d" % g_color_dict[g_color_hsv][0][0]
            S_min = "%03d" % g_color_dict[g_color_hsv][0][1]
            V_min = "%03d" % g_color_dict[g_color_hsv][0][2]
            H_max = "%03d" % g_color_dict[g_color_hsv][1][0]
            S_max = "%03d" % g_color_dict[g_color_hsv][1][1]
            V_max = "%03d" % g_color_dict[g_color_hsv][1][2]
            checksumstr = "%02x" % checksum
            data = "$1918" + H_min+S_min+V_min+H_max+S_max+V_max + checksumstr + "#"
            g_socket.send(data.encode(encoding="utf-8"))
        elif g_colorstudy==4:
            g_color_hsv="yellow"
            checksum = (19 + 18 + g_color_dict[g_color_hsv][0][0] +
                        g_color_dict[g_color_hsv][0][1] +
                        g_color_dict[g_color_hsv][0][2] +
                        g_color_dict[g_color_hsv][1][0] +
                        g_color_dict[g_color_hsv][1][1] +
                        g_color_dict[g_color_hsv][1][2]) & 0xff
            H_min = "%03d" % g_color_dict[g_color_hsv][0][0]
            S_min = "%03d" % g_color_dict[g_color_hsv][0][1]
            V_min = "%03d" % g_color_dict[g_color_hsv][0][2]
            H_max = "%03d" % g_color_dict[g_color_hsv][1][0]
            S_max = "%03d" % g_color_dict[g_color_hsv][1][1]
            V_max = "%03d" % g_color_dict[g_color_hsv][1][2]
            checksumstr = "%02x" % checksum
            data = "$1918" + H_min+S_min+V_min+H_max+S_max+V_max + checksumstr + "#"
            g_socket.send(data.encode(encoding="utf-8"))
#         print("mode : ",mode)
#         print("g_color_hsv : ",g_color_hsv)
        if mode == 0:
            g_colorstudyMode = 'colorstudyMode'
            Arm.Arm_serial_servo_write6_array(joints_init_down, 1000)
        elif mode == 1:
            g_colorstudyMode = 'colorstudyModeOK'
            Arm.Arm_Buzzer_On(1)
            #颜色HSV写入配置文件
        elif mode == 2:
            g_colorstudyMode = 'colorstudyMode_HSV'
        elif mode == 3:
            g_colorstudyMode = 'colorstudyModeOK_HSV'
            Arm.Arm_Buzzer_On(1)
        print(g_colorstudyMode)
    elif cmd == '1A': #向导界面：颜色校准HSV
        g_HSVindex = int(str[5:6])
        try:
            hsv = int(str[6:9])
        except:
            print("data error")
            return
        if g_color_hsv != None:
            if g_color_hsv == "red":
                g_HSV_Red[g_HSVindex-1] = hsv
                g_color_dict_temp[g_color_hsv] = ((g_HSV_Red[0], g_HSV_Red[1] , g_HSV_Red[2] ), (g_HSV_Red[3], g_HSV_Red[4], g_HSV_Red[5]))
            elif g_color_hsv == "green":
                g_HSV_Green[g_HSVindex-1] = hsv
                g_color_dict_temp[g_color_hsv] = ((g_HSV_Green[0], g_HSV_Green[1] , g_HSV_Green[2] ), (g_HSV_Green[3], g_HSV_Green[4], g_HSV_Green[5]))
            elif g_color_hsv == "blue":
                g_HSV_Blue[g_HSVindex-1] = hsv
                g_color_dict_temp[g_color_hsv] = ((g_HSV_Blue[0], g_HSV_Blue[1] , g_HSV_Blue[2] ), (g_HSV_Blue[3], g_HSV_Blue[4], g_HSV_Blue[5]))
            elif g_color_hsv == "yellow":
                g_HSV_Yellow[g_HSVindex-1] = hsv
                g_color_dict_temp[g_color_hsv] = ((g_HSV_Yellow[0], g_HSV_Yellow[1] , g_HSV_Yellow[2] ), (g_HSV_Yellow[3], g_HSV_Yellow[4], g_HSV_Yellow[5]))
            #print(g_color_dict_temp)
        
        
#socket TCP通信建立
def start_tcp_server(ip, port):
    global g_init, g_socket
    g_init = True
    print('start_tcp_server')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((ip, port))
    sock.listen(5)
    
    while True:
        if g_init == False:
            break
        print("开始等待客户连接")
        times = 0
        conn, address = sock.accept()
        print("连接客户：", address)
        g_socket = conn
        while True:
            if times == 0:
                data = "$0102100d#"
                g_socket.send(data.encode(encoding="utf-8"))
                times = 1
            try:
                cmd = g_socket.recv(1024).decode(encoding="utf-8") 
            except:
                break
            if not cmd:
                break
            print('   [-]cmd:', cmd, len(cmd))
            index1 = cmd.find("$")
            index2 = cmd.find("#")
            if index1 >= 0 and index2 > index1:
                Analysis(g_socket, cmd[index1:index2+1])
        g_socket.close() #
        #此处连续解包数据
#         handleTid = threading.Thread(target = message_handle, args = [conn])
#         handleTid.setDaemon(True)
#         handleTid.start()
    print("start_tcp_server close")
    closeTid = threading.Thread(target = waitClose, args = [conn])
    closeTid.daemon = True
    closeTid.start()

#关闭socket
def waitClose(sock):
    global g_init
    g_init = False
    time.sleep(10)
    sock.close()

#颜色分拣进程
def waitColorSort(pos,xy):
    global g_mode, g_socket
    Arm_identify.identify_grap(pos,xy)
    g_mode = 'Standard'
    data = "$0D02000F#"
    g_socket.send(data.encode(encoding="utf-8"))

#颜色堆叠进程
def waitColorStack(pos,xy):
    global g_mode, g_socket
    Arm_stack.stacking_grap(pos,xy)
    g_mode = 'Standard'
    data = "$0E020010#"
    g_socket.send(data.encode(encoding="utf-8"))
    
#垃圾分类进程
def waitGarbage(pos,xy):
    global g_mode, g_socket
    Arm_garbage.garbage_grap(pos,xy)
    g_mode = 'Standard'
    data = "$12020014#"
    g_socket.send(data.encode(encoding="utf-8"))
    
def waitSnakeOuthole(color, snake_msg):
    Arm_snake_ctrl.snake_main(color, snake_msg)
    
# 根据状态机来运行程序包含视频流返回
def mode_handle():
    global color_lower
    global color_uppersingle_rubbishOK
    global g_servormode, g_mode, g_socket, g_calibrateMode, g_colorDisplay, g_colorstudyMode, g_calibrateXY
    global g_color_hsv, g_calibrateThreshold, g_color_dict
    global g_camera
    global g_wifi
    
    # 初始化文字
    cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
    cal_dp=[]
    hsv_range=()
    pos={}
    get_color_hsv=()
    garbage_num = 'None'   
    garbage_class = 'None' 
    garbage_num_temp=" "
#    camservoInitFunction()
    while True:
        
        if g_wifi == False:
            frame = g_camera.get_frame()
        else :
            return
        #frame = g_camera.value
        #cv2.putText(img, str(i), (123,456)), font, 2, (0,255,0), 3) 
        #各参数依次是：图片，添加的文字，左上角坐标，字体，字体大小，颜色，字体粗细
        cv2.putText(frame, g_mode, (5,25), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,0), 2)
        # frame  = cv2AddChineseText(frame, g_mode, (5,25),(255, 0 , 0), 16)
        #cv2.putText(frame, g_mode, (5,25), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,0), 1)
        #颜色学习模式
        if g_colorstudyMode == 'colorstudyMode': #进入颜色学习模式
            #print("calibrateMode")
            frame, get_color_hsv = calibration.get_hsv(frame)
        elif g_colorstudyMode == 'colorstudyModeOK': #颜色学习确定
            g_color_dict[g_color_hsv] = get_color_hsv
            #写入配置文件HSV
            cf.set('HSV', 'g_HSV_Red', str(", ".join(repr(e) for e in g_color_dict["red"])).replace('(', '').replace(')', ''))
            cf.set('HSV', 'g_HSV_Green', str(", ".join(repr(e) for e in g_color_dict["green"])).replace('(', '').replace(')', ''))
            cf.set('HSV', 'g_HSV_Blue', str(", ".join(repr(e) for e in g_color_dict["blue"])).replace('(', '').replace(')', ''))
            cf.set('HSV', 'g_HSV_Yellow', str(", ".join(repr(e) for e in g_color_dict["yellow"])).replace('(', '').replace(')', ''))
            with open(file_path, 'w')as conf: cf.write(conf)
            if g_color_hsv=="red":
                g_colorstudyMode = 'colorstudyMode'
                g_color_hsv='green'
            elif g_color_hsv=="green":
                g_colorstudyMode = 'colorstudyMode'
                g_color_hsv='blue'
            elif g_color_hsv=="blue":
                g_colorstudyMode = 'colorstudyMode'
                g_color_hsv='yellow'
            elif g_color_hsv=="yellow":
                g_colorstudyMode ='None'
                g_color_hsv='None'
        elif g_colorstudyMode =='colorstudyModeCancel': #颜色学习模式退出
            g_color_dict =  {"red": ((0, 150, 160), (10, 255, 255)),
                              "green": ((35, 80, 36), (77, 255, 255)),
                              "blue": ((110, 124, 100), (124, 253, 255)),
                              "yellow": ((26, 98, 130), (34, 255, 255))}
        elif g_colorstudyMode == 'colorstudyMode_HSV': #进入颜色专业校准模式 colorstudyMode_HSV
            hsv_msg = {} 
            hsv_msg[g_color_hsv] = g_color_dict_temp[g_color_hsv]
            if g_colorDisplay == 'Normal':
                frame, _ = colorCalibration.get_contours(frame, g_color_hsv,hsv_msg[g_color_hsv],g_color_dict)
            else:
                _, frame = colorCalibration.get_contours(frame, g_color_hsv,hsv_msg[g_color_hsv],g_color_dict)
        elif g_colorstudyMode == 'colorstudyModeOK_HSV': #颜色专业校准确定
            if g_color_hsv!=None:
                g_color_dict[g_color_hsv] = g_color_dict_temp[g_color_hsv]
                #写入配置文件HSV
                cf.set('HSV', 'g_HSV_Red', str(", ".join(repr(e) for e in g_color_dict["red"])).replace('(', '').replace(')', ''))
                cf.set('HSV', 'g_HSV_Green', str(", ".join(repr(e) for e in g_color_dict["green"])).replace('(', '').replace(')', ''))
                cf.set('HSV', 'g_HSV_Blue', str(", ".join(repr(e) for e in g_color_dict["blue"])).replace('(', '').replace(')', ''))
                cf.set('HSV', 'g_HSV_Yellow', str(", ".join(repr(e) for e in g_color_dict["yellow"])).replace('(', '').replace(')', ''))
                with open(file_path, 'w')as conf:
                    cf.write(conf)
                print(g_color_dict)
            g_colorstudyMode ='None'

        
        #标定模式
        if g_calibrateMode == 'calibrateMode': #进入标定模式
            #print("calibrateMode")
            _, frame = calibration.calibration_map(frame, g_calibrateXY, g_calibrateThreshold)
        elif g_calibrateMode == 'calibrateOK': #标定确定
            cal_dp, frame = calibration.calibration_map(frame, g_calibrateXY, g_calibrateThreshold)
            g_calibrateMode ='None'
            #写入配置文件g_calibrateXY, g_calibrateThreshold
            cf.set('calibrateThreshold', 'g_calibratethreshold', str(g_calibrateThreshold))
            cf.set('calibrateXY', 'g_calibrateXY', str(", ".join(repr(e) for e in g_calibrateXY)))
            with open(file_path, 'w')as conf:
                cf.write(conf)
        elif g_calibrateMode =='calibrateCancel': #标定退出
            cal_dp=[]
        if len(cal_dp)!=0:
            frame = calibration.Perspective_transform(cal_dp, frame)
        
        
        # 颜色分拣玩法
        if g_mode == 'colorsortplan': #传颜色分拣顺序并实时显示识别效果
            frame, pos = Arm_identify.select_color(frame,g_color_dict, g_colororder)
        elif g_mode == 'colorsortstart': #开始分拣
#             print("start colorsort")
#             print(pos)
            if len(pos)!=0: 
                closeTid = threading.Thread(target = waitColorSort, args = [pos,g_calibrateXY])
                closeTid.daemon = True
                closeTid.start()
                pos={}
                g_mode = 'ColorSorting ...'

        # 颜色堆叠玩法
        elif g_mode == 'colorstackplan': #传颜色堆叠顺序并实时显示识别效果
            frame, pos = Arm_stack.select_color(frame, g_color_dict, g_colororder)
        elif g_mode == 'colorstackstart': #开始堆叠
            if len(pos)!=0: 
                closeTid = threading.Thread(target = waitColorStack, args = [pos,g_calibrateXY])
                closeTid.daemon = True
                closeTid.start()
                pos={}
                g_mode = 'ColorStacking ...'
        #颜色互动玩法
        elif g_mode == 'YouputWecatchMode': #你放我抓
            frame= Arm_sorting.Sorting_grap(frame,g_color_dict)
        elif g_mode == 'SnakeoutholeMode1': #引蛇出洞
            current_color_hsv = {'red': g_color_dict['red']}
            frame,snake_msg = Arm_snake_target.target_run(frame,current_color_hsv)
            # if len(snake_msg) == 1 :
            closeTid = threading.Thread(target = waitSnakeOuthole, args = ("red", snake_msg,))
            closeTid.daemon = True
            closeTid.start()
        elif g_mode == 'SnakeoutholeMode2': #引蛇出洞
            current_color_hsv = {'green': g_color_dict['green']}
            frame,snake_msg = Arm_snake_target.target_run(frame, current_color_hsv)
            # if len(snake_msg) == 1 :
            closeTid = threading.Thread(target = waitSnakeOuthole, args = ("green", snake_msg,))
            closeTid.daemon = True
            closeTid.start()
        elif g_mode == 'SnakeoutholeMode3': #引蛇出洞
            current_color_hsv = {'blue': g_color_dict['blue']}
            frame,snake_msg = Arm_snake_target.target_run(frame,current_color_hsv)
            # if len(snake_msg) == 1 :
            closeTid = threading.Thread(target = waitSnakeOuthole, args = ("blue", snake_msg,))
            closeTid.daemon = True
            closeTid.start()
        elif g_mode == 'SnakeoutholeMode4': #引蛇出洞
            current_color_hsv = {"yellow": g_color_dict["yellow"]}
            # print(current_color_hsv)
            frame,snake_msg = Arm_snake_target.target_run(frame, current_color_hsv)
            # if len(snake_msg) == 1 :
            closeTid = threading.Thread(target = waitSnakeOuthole, args = ("yellow", snake_msg,))
            closeTid.daemon = True
            closeTid.start()
        #颜色抓取
        elif g_mode == 'ColorRecogFllowCatchMode': 
            frame = Arm_color_grab.start_grab(frame)  
        #手势互动玩法            
        elif g_mode == 'GestureActionMode':
            frame, ges = Arm_gesture_action.start_gesture(frame)
        #手势堆叠    
        elif g_mode == 'GestureStackMode': 
            frame = Arm_gesture_stack.start_gesture(frame)
        # 垃圾分类玩法
        elif g_mode == 'rubbishplan': #垃圾识别
            frame, pos = Arm_garbage.garbage_run(frame)
        elif g_mode == 'rubbishOK': #垃圾分类
            #print("start colorsort")
            print(pos)
            if len(pos)!=0: 
                closeTid = threading.Thread(target = waitGarbage, args = [pos,g_calibrateXY])
                closeTid.daemon = True
                closeTid.start()
            pos={}
            g_mode='Standard'
        elif g_mode == 'single_rubbishplan': # 单个垃圾识别
            frame,garbage_num,garbage_class=Arm_single_garbage.single_garbage_run(frame)
            if garbage_num!='None':
                data = "$1202"+garbage_num+garbage_class+"XX#"
                g_socket.send(data.encode(encoding="utf-8"))
            
        # 颜色追踪玩法
        elif g_mode == 'colortrack1':
            frame = Arm_color_follow.follow_function(frame, 'red',g_color_dict['red'])
        elif g_mode == 'colortrack2':
            frame = Arm_color_follow.follow_function(frame, 'green',g_color_dict['green'])
        elif g_mode == 'colortrack3':
            frame = Arm_color_follow.follow_function(frame, 'blue',g_color_dict['blue'])
        elif g_mode == 'colortrack4':
            frame = Arm_color_follow.follow_function(frame, 'yellow',g_color_dict['yellow'])
        elif g_mode == 'colortrackstudy':
            frame, hsv_range = Arm_color_follow.get_hsv(frame)
        elif g_mode == 'colortrackstudyOK':
#             print(hsv_range)
#             (hsvlow, hsvhigh) = hsv_range
#             hsv_range = (hsvlow, hsvhigh)
            #hsv_range =  ((0, 43, 46),(10, 255, 255))
            frame = Arm_color_follow.learning_follow(frame, hsv_range)
#         print("------------------222")
        # 人脸追踪玩法
        elif g_mode == 'facefunction':
            frame = Arm_face_follow.follow_function(frame)
        imgencode = cv2.imencode('.jpg', frame)[1]
        imgencode = imgencode.tobytes()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + imgencode + b'\r\n')

        time.sleep(0.006)
    del(g_camera)
    
@app.route('/')
def index():
    return render_template('index.html')

def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(mode_handle(),
          mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/init')
def init():
    global g_init
    if g_init == False:
        while(True):
            ip = getLocalip()
            print(ip)
            if(ip == "x.x.x.x"):
                continue
            if(ip != "x.x.x.x"):
                break
        print('creat start_tcp_server')
        tid=threading.Thread(target=start_tcp_server, args=(ip, 6000,))
        tid.daemon = True
        tid.start()

    print('init socket!!!!!!!!!')
    return render_template('init.html')

# wifi 配网的线程。
def thread_wifi():
    global g_camera, g_wifi, g_init
    g_wifi = False
    while True:
        if Arm_wifi.read_mode() == False:
            time.sleep(0.1)
            continue
        else:
            # print('111')
            g_wifi = True
            Arm.Arm_Buzzer_On(1)
            Arm.Arm_serial_servo_write6_array(joints_init_front, 1000)
        ip_init='0'
        while True:
            # print('222')
            if g_wifi == True:
                frame = g_camera.get_frame()
                _, ip_init = Arm_wifi.connect(frame)
            if ip_init == '0.0.0.0':
                Arm_wifi.set_mode(False)
                g_wifi = False
                g_init = False
                break
                
            time.sleep(1)

if __name__ == '__main__':
    global g_camera
    g_camera = VideoCamera()

    for i in range(3):
        Arm.Arm_Buzzer_On(1) 
        time.sleep(0.23)
    app.run(host='0.0.0.0', port=6500, debug=False)

