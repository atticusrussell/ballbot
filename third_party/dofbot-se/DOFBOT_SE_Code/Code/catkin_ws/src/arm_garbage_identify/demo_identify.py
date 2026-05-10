#!/usr/bin/env python
# coding: utf-8
import cv2 as cv
from garbage_identify import garbage_identify

if __name__ == '__main__':
    gar = garbage_identify()
    capture = cv.VideoCapture(0)
    while capture.isOpened():
        _, image = capture.read()
        image, msg = gar.garbage_run(image)
        gar.garbage_grap(msg)
        cv.imshow("image", image)
        cv.waitKey(10)
    cv.destroyAllWindows()
