# !/usr/bin/env python
# encoding:utf-8
import cv2 as cv

if __name__ == '__main__':
    capture = cv.VideoCapture(0)
    while 1:
        _, img = capture.read()
        img = cv.resize(img, (640, 480), )
        cv.imshow("img", img)
        action = cv.waitKey(10) & 0xff
        if action == 27:
            break
    cv.destroyAllWindows()