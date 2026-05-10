import sys
sys.path.append('/home/jetson/dofbot_pro/dofbot_color_grab/scripts')
import cv2 as cv
import cv2
import threading
from time import sleep
from dofbot_utils.dofbot_config import *
import ipywidgets as widgets
from IPython.display import display
from speech_color_sorting import speech_color_sorting
from dofbot_utils.robot_controller import Robot_Controller
from dofbot_utils.fps import FPS
from Speech_Lib import Speech
import os
import time
'''mySpeech = Speech()
time.sleep(.1)
os.system("mpg123 /home/jetson/speech_music/0.mp3")'''
robot = Robot_Controller()
robot.move_look_map()
fps = FPS()

# 创建实例
sorting = speech_color_sorting()
# 初始化模式

# 颜色HSV阈值
color_hsv  = {"red"   : ((0, 43, 46), (10, 255, 255)),
              "green" : ((35, 43, 46), (77, 255, 255)),
              "blue"  : ((100, 43, 46), (124, 255, 255)),
              "yellow": ((26, 43, 46), (34, 255, 255))}
# HSV参数路径
HSV_path="/home/jetson/dofbot_pro/dofbot_color_identify/scripts/HSV_config.txt"
# 读取HSV配置文件,更新HSV值
try: read_HSV(HSV_path,color_hsv)
except Exception: print("Read HSV_config Error!!!")

def camera():
    model = 'General'
    # 打开摄像头
    capture = cv.VideoCapture(0, cv.CAP_V4L2)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        try:
            key = cv2.waitKey(10)
            if key == 81 or key == 113:
                model = 'Exit'
            # 读取相机的每一帧
            _, img = capture.read()
            # 获得运动信息
            img = sorting.Sorting_grap(img, color_hsv)
            if model == 'Exit':
                cv.destroyAllWindows()
                capture.release()
                break
            cv2.imshow("res_image", img)

            # 添加文字
            #imgbox.value = cv.imencode('.jpg', img)[1].tobytes()
        except KeyboardInterrupt:capture.release()
            
threading.Thread(target=camera, ).start()
