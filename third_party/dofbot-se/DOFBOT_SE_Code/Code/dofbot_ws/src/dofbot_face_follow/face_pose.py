# !/usr/bin/env python
# coding: utf-8
import cv2 as cv


class face_follow:
    def __init__(self):

        # Create an instance of OpenCV's joint classifier
        # 创建opencv的联级分类器的实例
        self.faceDetect = cv.CascadeClassifier("haarcascade_frontalface_default.xml")

    def face_filter(self, faces):
        '''
        Filter the face
        对人脸进行一个过滤
        '''
        if len(faces) == 0: return None
        # At present, we are looking for the face with the largest area in the pictur
        # 目前找的是画面中面积最大的人脸
        max_face = max(faces, key=lambda face: face[2] * face[3])
        (x, y, w, h) = max_face
        # Set the minimum threshold of face detection
        # 设置人脸检测最小阈值
        if w < 10 or h < 10: return None
        return max_face

    def follow_function(self, img):

        img = cv.resize(img, (640, 480))
        # Copy the original image to avoid interference during processing
        # 复制原始图像,避免处理过程中干扰
        img = img.copy()
        # Convert image to grayscale
        # 将图像转为灰度图
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Face detection
        # 检测人脸
        faces = self.faceDetect.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
        if len(faces) != 0:
            face = self.face_filter(faces)
            # Face filtering
            # 人脸过滤
            (x, y, w, h) = face
            # Draw a rectangle on the original color map
            # 在原彩图上绘制矩形
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 4)
            cv.putText(img, 'Person', (280, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (105, 105, 105), 2)
            point_x = x + w / 2
            point_y = y + h / 2

            print("x= ",point_x)
            print("y= ",point_y)
            
        return img
