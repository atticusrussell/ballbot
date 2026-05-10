#!/usr/bin/env python
# coding: utf-8
from face_follow import face_follow
import cv2 as cv

if __name__ == '__main__':
    capture = cv.VideoCapture(0)
    follow = face_follow()
    # capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    # capture.set(cv.CAP_PROP_BRIGHTNESS, 30) #设置亮度 -64 - 64  0.0
    # capture.set(cv.CAP_PROP_CONTRAST, 50) #设置对比度 -64 - 64  2.0
    # capture.set(cv.CAP_PROP_EXPOSURE, 156) #设置曝光值 1.0 - 5000  156.0
    while capture.isOpened():
        _, img = capture.read()
        img = follow.run(img)
        cv.line(img, (320, 0), (320, 480), (255, 0, 0), 1)
        cv.line(img, (0, 240), (680, 240), (255, 0, 0), 1)
        cv.imshow("img", img)
        action = cv.waitKey(10) & 0xff
        if action == ord('q') or action == 27:
            cv.destroyAllWindows()
            capture.release()
            break
    cv.destroyAllWindows()
    capture.release()
