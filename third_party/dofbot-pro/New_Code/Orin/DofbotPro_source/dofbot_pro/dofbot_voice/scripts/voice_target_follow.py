import sys
sys.path.append('/home/jetson/dofbot_pro/dofbot_color_follow/scripts')
sys.path.append('/home/jetson/dofbot_pro/dofbot_face_follow/scripts')
import time
import cv2 as cv
import cv2 
import threading
import random
from time import sleep
import ipywidgets as widgets
from IPython.display import display
from color_follow import color_follow
from face_follow import Face_Follow
from dofbot_utils.dofbot_config import *
from Speech_Lib import Speech
import Arm_Lib
Arm = Arm_Lib.Arm_Device()
joints_0 = [90, 135, 20, 25, 90, 30]
Arm.Arm_serial_servo_write6_array(joints_0, 1000)

mySpeech = Speech()
follow = color_follow()
follow2 = Face_Follow()
# 初始化模式
model = 'General'
# 初始化HSV_learning值
HSV_learning = ()
# 初始化HSV值
color_hsv = {"red": ((170, 124, 134), (229, 242, 255)),
             "green": ((54, 113, 64), (75, 255, 255)),
             "blue": ((102, 150, 124), (124, 253, 255)),
             "yellow": ((22, 125, 130), (47, 255, 255))}
# 设置随机颜色
color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
# HSV参数路径
HSV_path="/home/jetson/dofbot_pro/dofbot_color_follow/scripts/HSV_config.txt"
try: read_HSV(HSV_path,color_hsv)
except Exception: print("Read HSV_config Error !!!")


def camera():
    global HSV_learning,model
    # 打开摄像头
    capture = cv.VideoCapture(0, cv.CAP_V4L2)
    capture.set(3, 640)
    capture.set(4, 480)
    capture.set(5, 30)  #设置帧率
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        try:
            
            time.sleep(0.1)
            # 读取相机的每一帧
            _, img = capture.read()
            key = cv2.waitKey(10)
            # 统一图像大小
            # img = cv.resize(img, (640, 480))
            result = mySpeech.speech_read()
            if result == 73:
                choose_color = 'red'
                model = 'color_follow'
                mySpeech.void_write(71)#等待当前语句播报结束
                time.sleep(0.1) 
            elif result == 72:
                choose_color = 'yellow'
                model = 'color_follow'
                mySpeech.void_write(71)#等待当前语句播报结束
                time.sleep(0.1)
            elif result == 74:
                choose_color = 'green'
                model = 'color_follow'
                mySpeech.void_write(71)#等待当前语句播报结束
                time.sleep(0.1)  
            elif result == 75:
                choose_color = 'blue'
                model = 'color_follow'
                mySpeech.void_write(71)#等待当前语句播报结束
                time.sleep(0.1)     
            elif result == 71:
                model = 'follow2'
                mySpeech.void_write(71)#等待当前语句播报结束
                time.sleep(0.1)   
            elif result == 76:
                model = 'General'
                mySpeech.void_write(76)#等待当前语句播报结束
                time.sleep(0.1)
            
            if model == 'color_follow':
                img = follow.follow_function(img, color_hsv[choose_color])
                # 添加文字
                cv.putText(img, choose_color, (int(img.shape[0] / 2), 50), cv.FONT_HERSHEY_SIMPLEX, 2, color[random.randint(0, 254)], 2)
            if model == 'follow2':
                img, pos = follow2.follow_function(img)
            if model == 'learning_color':
                img,HSV_learning = follow.get_hsv(img)
            if model == 'learning_follow' :
                if len(HSV_learning)!=0:
                    print(HSV_learning)
                    img = follow.learning_follow(img, HSV_learning)
                    # 添加文字
                    cv.putText(img,'LeColor', (240, 50), cv.FONT_HERSHEY_SIMPLEX, 1, color[random.randint(0, 254)], 1)
            if model == 'Exit':
                capture.release()
                break
            cv2.imshow("res_image", img)
        except KeyboardInterrupt:
            capture.release()
            break
        except Exception as e:
            print("error:", e)
            break
threading.Thread(target=camera, ).start()



