#!/usr/bin/env python
# coding: utf-8
import cv2
from gesture_action import Gesture_Action

if __name__ == '__main__':
    gesture = Gesture_Action()
    capture = cv2.VideoCapture(0)
    capture.set(3, 640)
    capture.set(4, 480)
    capture.set(5, 30)
    while capture.isOpened():
        _, img = capture.read()
        img = gesture.process(img)
        cv2.imshow("img", img)
        action = cv2.waitKey(10) & 0xff
        if action == ord('q') or action == 27:
            break
    cv2.destroyAllWindows()
    capture.release()