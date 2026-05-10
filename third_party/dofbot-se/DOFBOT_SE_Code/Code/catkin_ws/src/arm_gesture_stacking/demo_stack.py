#!/usr/bin/env python
# coding: utf-8
import cv2
from gesture_stack import gesture_stack

if __name__ == '__main__':
    gesture = gesture_stack()
    capture = cv2.VideoCapture(0)
    capture.set(3, 640)
    capture.set(4, 480)
    capture.set(5, 30)  #设置帧率
    # capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    # capture.set(cv.CAP_PROP_BRIGHTNESS, 30) #设置亮度 -64 - 64  0.0
    # capture.set(cv.CAP_PROP_CONTRAST, 50) #设置对比度 -64 - 64  2.0
    # capture.set(cv.CAP_PROP_EXPOSURE, 156) #设置曝光值 1.0 - 5000  156.0
    try:
        while capture.isOpened():
            _, img = capture.read()
            img = gesture.start_gesture(img)
            cv2.imshow("img", img)
            action = cv2.waitKey(10) & 0xff
            if action == ord('q') or action == 27:
                cv2.destroyAllWindows()
                capture.release()
                break
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        capture.release()
        print(" Program closed! ")
        pass
    cv2.destroyAllWindows()
    capture.release()