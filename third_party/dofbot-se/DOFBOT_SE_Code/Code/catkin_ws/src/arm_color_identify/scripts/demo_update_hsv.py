# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
from Calibration import update_hsv

color_hsv = {"red": ((2, 100, 60), (11, 255, 200)),
             "green": ((55, 80, 20), (78, 255, 86)),
             "blue": ((110, 100, 30), (125, 255, 200)),
             "yellow": ((26, 100, 100), (32, 232, 230))}
HSV_min=[0,100,120]
HSV_max=[10,255,255]

def update_H_min(value):
    global HSV_min
    HSV_min[0] = value

def update_S_min(value):
    global HSV_min
    HSV_min[1]=value

def update_V_min(value):
    global HSV_min
    HSV_min[2]=value

def update_H_max(value):
    global HSV_max
    HSV_max[0]=value

def update_S_max(value):
    global HSV_max
    HSV_max[1]=value

def update_V_max(value):
    global HSV_max
    HSV_max[2]=value


cv.namedWindow('img', flags=cv.WINDOW_NORMAL | cv.WINDOW_KEEPRATIO | cv.WINDOW_GUI_EXPANDED)
cv.createTrackbar('H_min', 'img', 0, 255, update_H_min)
cv.setTrackbarPos('H_min', 'img', HSV_min[0])
cv.createTrackbar('S_min', 'img', 0, 255, update_S_min)
cv.setTrackbarPos('S_min', 'img', HSV_min[1])
cv.createTrackbar('V_min', 'img', 0, 255, update_V_min)
cv.setTrackbarPos('V_min', 'img', HSV_min[2])
cv.createTrackbar('H_max', 'img', 0, 255, update_H_max)
cv.setTrackbarPos('H_max', 'img', HSV_max[0])
cv.createTrackbar('S_max', 'img', 0, 255, update_S_max)
cv.setTrackbarPos('S_max', 'img', HSV_max[1])
cv.createTrackbar('V_max', 'img', 0, 255, update_V_max)
cv.setTrackbarPos('V_max', 'img', HSV_max[2])

if __name__ == '__main__':
    hsv_update = update_hsv()
    capture = cv.VideoCapture(0)
    while capture.isOpened():
        _, img = capture.read()
        hsv_name = "red"
        color_hsv[hsv_name] = ((HSV_min[0], HSV_min[1], HSV_min[2]), (HSV_max[0], HSV_max[1], HSV_max[2]))
        img, binary = hsv_update.get_contours(img, hsv_name,color_hsv[hsv_name])
        cv.imshow("img", img)
        cv.imshow("binary", binary)
        action = cv.waitKey(10) & 0xff
        if action == 32:
            print ('写入配置文件')
        if action == 27 or action == ord('q'):
            capture.release()
            cv.destroyAllWindows()
            break
    capture.release()
    cv.destroyAllWindows()
