#coding=utf-8
# camera.py
import cv2
 
class VideoCamera(object):
    def __init__(self):
        # Using OpenCV to capture from device 0. If you have trouble capturing
        # from a webcam, comment the line below out and use a video file
        # instead.
        self.video = cv2.VideoCapture(0,cv2.CAP_V4L2)
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.video.set(cv2.CAP_PROP_BRIGHTNESS, 30) #设置亮度 -64 - 64  0.0
        self.video.set(cv2.CAP_PROP_CONTRAST, 50) #设置对比度 -64 - 64  2.0
        self.video.set(cv2.CAP_PROP_EXPOSURE, 156) #设置曝光值 1.0 - 5000  156.0
        self.video.set(3, 640)
        self.video.set(4, 480)
        self.video.set(5, 30)  # 设置帧率
        # If you decide to use video.mp4, you must have this file in the folder
        # as the main.py.
        # self.video = cv2.VideoCapture('video.mp4')
 
    def __del__(self):
        self.video.release()
 
    def get_frame(self):
        success, image = self.video.read()
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        if(success == False):
            print ("Read Error!")
            return bytes({1});
        return image
    
    def get_frame2(self):
        success, image = self.video.read()
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        if(success == False):
            #print ("Read Error!")
            return bytes({1});
       
        ret, jpeg = cv2.imencode('.jpg', image)
        
        # 对于 python2.7 或者低版本的 numpy 请使用 jpeg.tostring()
        return jpeg.tobytes()
