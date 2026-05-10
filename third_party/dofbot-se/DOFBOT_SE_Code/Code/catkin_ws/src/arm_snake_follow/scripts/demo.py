# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
import threading
from snake_target import snake_target
from snake_ctrl import snake_ctrl

color_hsv = {"red": ((0, 25, 90), (10, 255, 255)),
             "green": ((53, 36, 40), (80, 255, 255)),
             "blue": ((116, 80, 90), (130, 255, 255)),
             "yellow": ((25, 20, 55), (50, 255, 255))}
if __name__ == '__main__':
    snake_target = snake_target()
    snake_ctrl = snake_ctrl()
    capture = cv.VideoCapture(0)
    # capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    # capture.set(cv.CAP_PROP_BRIGHTNESS, 30) #设置亮度 -64 - 64  0.0
    # capture.set(cv.CAP_PROP_CONTRAST, 50)   #设置对比度 -64 - 64  2.0
    # capture.set(cv.CAP_PROP_EXPOSURE, 156)  #设置曝光值 1.0 - 5000  156.0
    while capture.isOpened():
        _, img = capture.read()
        img, msg = snake_target.target_run(img, color_hsv)
        if len(msg) == 1:
            threading.Thread(target=snake_ctrl.snake_ctrl, args=("red", msg,)).start()
        cv.imshow("img", img)
        action = cv.waitKey(10) & 0xff
        if action == ord('q') or action == 27:
            cv.destroyAllWindows()
            capture.release()
            break
    cv.destroyAllWindows()
    capture.release()
