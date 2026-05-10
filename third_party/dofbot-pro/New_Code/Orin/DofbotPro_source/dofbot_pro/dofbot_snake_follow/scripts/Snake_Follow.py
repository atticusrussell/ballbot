#!/usr/bin/env python
# coding: utf-8
import cv2 as cv
import threading
from time import sleep
import random
from snake_target import snake_target
from snake_ctrl import snake_ctrl
from dofbot_utils.dofbot_config import *
import Arm_Lib

# 定义一个标志位用于控制线程的运行
running = True

Arm = Arm_Lib.Arm_Device()
joints_0 = [90, 135, 0, 45, 0, 180]
Arm.Arm_serial_servo_write6_array(joints_0, 1000)

snake_target = snake_target()
snake_ctrl = snake_ctrl()
model = 'General'
# 定义颜色列表
color_list = ["red", "green", "blue", "yellow"]
# 初始化当前颜色索引
current_color_index = 3  # 初始化为黄色
color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
color_hsv = {
    "red": ((0, 43, 46), (10, 255, 255)),
    "green": ((35, 43, 46), (77, 255, 255)),
    "blue": ((100, 43, 46), (124, 255, 255)),
    "yellow": ((26, 43, 46), (34, 255, 255))
}

HSV_path = "/home/jetson/dofbot_pro/dofbot_snake_follow/scripts/HSV_config.txt"
try:
    read_HSV(HSV_path, color_hsv)
except Exception:
    print("Read HSV_config Error!!!")

def camera():
    global running, current_color_index
    # 打开摄像头 Open camera
    capture = cv.VideoCapture(0)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # Be executed in loop when the camera is opened normally 
    # 当摄像头正常打开的情况下循环执行
    while running and capture.isOpened():
        try:
            _, img = capture.read()
            current_color = color_list[current_color_index]
            current_color_hsv = {current_color: color_hsv[current_color]}

            img, snake_msg = snake_target.target_run(img, current_color_hsv)
            if len(snake_msg) >= 1:
                current_color = color_list[current_color_index]
                threading.Thread(target=snake_ctrl.snake_main, args=(current_color, snake_msg,)).start()
            if model == 'Exit':
                running = False
                capture.release()
                break
            current_color = color_list[current_color_index]
            cv.putText(img, current_color, (int(img.shape[0] / 2), 50), cv.FONT_HERSHEY_SIMPLEX, 2, color[random.randint(0, 254)], 2)
            cv.imshow('Snake Follow', img)
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                running = False
                break
            elif key == ord('c'):
                # 切换颜色索引
                current_color_index = (current_color_index + 1) % len(color_list)
        except KeyboardInterrupt:
            running = False
            capture.release()

    cv.destroyAllWindows()

# 启动线程
camera_thread = threading.Thread(target=camera)
camera_thread.start()

# 等待线程结束
camera_thread.join()
