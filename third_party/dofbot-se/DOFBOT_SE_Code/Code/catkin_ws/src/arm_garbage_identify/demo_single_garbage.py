#!/usr/bin/env python
# coding: utf-8
import cv2 as cv
from single_garbage_identify import single_garbage_identify

# 创建获取目标实例
single_garbage = single_garbage_identify()
if __name__ == '__main__':
    # 打开摄像头
    capture = cv.VideoCapture(0)
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        # 读取相机的每一帧
        _, img = capture.read()
        # 统一图像大小
        img = cv.resize(img, (640, 480))
        # 水平翻转
        # img = cv.flip(img, -1)
        action = cv.waitKey(1) & 0xff
        # cv.rectangle(img, (50, 50), (300, 300), (105,105,105), 2)
        img = single_garbage.single_garbage_run(img)
        cv.imshow("img", img)
        # 退出程序 按键:"esc"
        if action == 27:
            capture.release()
            cv.destroyAllWindows()
            break
    capture.release()
    cv.destroyAllWindows()
