# !/usr/bin/env python
# coding: utf-8
import cv2 as cv


class Color_Position:
    def __init__(self):
        self.img = None
       
    def process(self, img, HSV_config):
        (color_lower, color_upper) = HSV_config
        # self.img = cv.resize(img, (640, 480), )
        self.img = img.copy()
        img1 = cv.GaussianBlur(self.img, (5, 5), 0)
        hsv = cv.cvtColor(img1, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, color_lower, color_upper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        mask = cv.GaussianBlur(mask, (5, 5), 0)
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        pos = None
        if len(cnts) > 0:
            cnt = max(cnts, key=cv.contourArea)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(cnt)
            if color_radius > 10:
                # Mark the detected color with the prototype coil
                # 将检测到的颜色用原形线圈标记出来
                cv.circle(self.img, (int(color_x), int(color_y)), int(color_radius), (255, 0, 255), 3)
                pos = (int(color_x), int(color_y))
        return self.img, pos


if __name__ == '__main__':
    from dofbot_utils.fps import FPS
    capture = cv.VideoCapture(0)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    position = Color_Position()
    fps = FPS()
    color_hsv = {"red": ((0, 25, 90), (10, 255, 255)),
                "green": ((53, 36, 40), (80, 255, 255)),
                "blue": ((110, 80, 90), (120, 255, 255)),
                "yellow": ((25, 20, 55), (50, 255, 255))}
    color_name = 'red'
    try:
        while capture.isOpened():
            ret, frame = capture.read()
            fps.update_fps()
            img, pos = position.process(frame, color_hsv[color_name])
            if pos is not None:
                print("x={}, y={}".format(pos[0], pos[1]))
            if cv.waitKey(1) & 0xFF == ord('q'): break
            fps.show_fps(img)
            cv.imshow('image', img)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("error:", e)
    capture.release()
    cv.destroyAllWindows()

