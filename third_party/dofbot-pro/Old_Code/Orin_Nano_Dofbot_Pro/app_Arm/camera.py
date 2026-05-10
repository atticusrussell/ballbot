#!/usr/bin/env python3
# coding=utf-8
import cv2 as cv
import time

# V1.0.3
class VideoCamera(object):

    def __init__(self, video_id=0, width=640, height=480, debug=False):
        self.__debug = debug
        self.__video_id = int(video_id)
        self.__state = False
        self.__width = width
        self.__height = height

        try:
            self.__video = cv.VideoCapture(self.__video_id)
            if self.__debug:
                print("video:", self.__video_id)

            # success, _ = self.__video.read()
            success = self.__video.isOpened()
            if not success:
                if self.__debug:
                    print("---------Camera Init Error!------------")
                return
            self.__state = True
            self.__config_camera()

            if self.__debug:
                print("---------Video:0x%02x Init OK!------------" % (self.__video_id))
        except:
            if self.__debug:
                    print("---------Error:No Video ------------")
            self.__state = False
        

    def __del__(self):
        if self.__debug:
            print("---------Del Camera!------------")
        self.__video.release()
        self.__state = False

    def __config_camera(self):
        self.__video.set(cv.CAP_PROP_FRAME_WIDTH, self.__width)
        self.__video.set(cv.CAP_PROP_FRAME_HEIGHT, self.__height)

    # 摄像头是否打开成功
    # Check whether the camera is enabled successfully
    def isOpened(self):
        if self.__state == False:
            return False
        return self.__video.isOpened()

    # 释放摄像头 Release the camera
    def clear(self):
        if self.isOpened():
            self.__video.release()
        self.__state = False


    # 重新连接摄像头 
    # Reconnect the camera
    def reconnect(self):
        try:
            self.clear()
            self.__video = cv.VideoCapture(self.__video_id)
            success, _ = self.__video.read()
            if not success:
                if self.__debug:
                    self.__state = False
                    print("---------Camera Reconnect Error!------------")
                return False
            if not self.__state:
                if self.__debug:
                    print("---------Video:0x%02x Reconnect OK!------------" % self.__video_id)
                self.__state = True
                self.__config_camera()
            return True
        except:
            self.__state = False
            return False


    # 获取摄像头的一帧图片 
    # Gets a frame of the camera
    def get_frame(self):
        success, image = self.__video.read()
        if not success:
            return success, bytes({1})
        return success, image

    # 获取摄像头的jpg图片 
    # Gets the JPG image of the camera
    def get_frame_jpg(self, text="", color=(0, 255, 0)):
        success, image = self.__video.read()
        if not success:
            return success, bytes({1})
        if text != "":
            # 各参数依次是：图片，添加的文字，左上角坐标，字体，字体大小，颜色，字体粗细
            # The parameters are: image, added text, top left coordinate, font, font size, color, font size  
            cv.putText(image, str(text), (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        success, jpeg = cv.imencode('.jpg', image)
        return success, jpeg.tobytes()


if __name__ == '__main__':
    camera = VideoCamera(debug=True)
    average = False
    m_fps = 0
    t_start = time.time()
    while camera.isOpened():
        if average:
            ret, frame = camera.get_frame()
            m_fps = m_fps + 1
            fps = m_fps / (time.time() - t_start)
            
        else:
            start = time.time()
            ret, frame = camera.get_frame()
            end = time.time()
            fps = 1 / (end - start)
        text="FPS:" + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0), 1)
        cv.imshow('frame', frame)

        k = cv.waitKey(1) & 0xFF
        if k == 27 or k == ord('q'):
            break
    del camera
    cv.destroyAllWindows()
