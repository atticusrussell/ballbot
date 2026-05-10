import sys
sys.path.append('/home/jetson/dofbot_pro/dofbot_garbage_yolov11')

import os
import cv2 as cv
import threading
from time import sleep
from speech_garbage import speech_garbage




# 创建获取目标实例
single_garbage = speech_garbage()
# 初始化模式
model = "General"

from dofbot_utils.robot_controller import Robot_Controller
robot = Robot_Controller()
robot.move_look_map()

from dofbot_utils.fps import FPS
fps = FPS()

def camera():
    # 打开摄像头
    capture = cv.VideoCapture(0, cv.CAP_V4L2)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        try:
            # 读取相机的每一帧
            _, img = capture.read()
            fps.update_fps()
            img = single_garbage.single_garbage_run(img)
            if model == 'Exit':
                cv.destroyAllWindows()
                capture.release()
                break
            fps.show_fps(img)
            cv.imshow("img", img)
            key = cv.waitKey(1)            
        except KeyboardInterrupt:capture.release()

threading.Thread(target=camera, ).start()