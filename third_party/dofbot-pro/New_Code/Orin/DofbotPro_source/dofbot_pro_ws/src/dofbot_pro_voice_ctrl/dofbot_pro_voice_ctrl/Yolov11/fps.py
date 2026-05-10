import time
import cv2

class FPS:
    def __init__(self):
        self.last_time = time.time()
        self.current_time = time.time()
        self.fps = 0.0
        self.temp_fps = 0

    # 更新FPS
    def update_fps(self):
        # self.current_time = time.time()
        # self.fps = 1 / (self.current_time - self.last_time)
        # self.last_time = self.current_time

        self.temp_fps = self.temp_fps + 1
        self.current_time = time.time()
        self.fps = self.temp_fps / (self.current_time - self.last_time + 1)
        if (self.current_time - self.last_time) >= 1:
            self.temp_fps = self.fps
            self.last_time = self.current_time
        return self.fps

    # 显示FPS
    def show_fps(self, img):
        font = cv2.FONT_HERSHEY_PLAIN
        line = cv2.LINE_AA
        text = 'FPS: {:.1f}'.format(self.fps)
        cv2.putText(img, text, (11, 20), font, 1.0, (0, 0, 0), 4, line)
        cv2.putText(img, text, (10, 20), font, 1.0, (255, 255, 255), 1, line)
        # cv2.putText(img, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        return img


