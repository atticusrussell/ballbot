import cv2
from PIL import ImageFont, ImageDraw, Image

class Object_recognition_identify:
    def __init__(self):
        
        self.net = cv2.dnn.readNet('yolov4-tiny.cfg', 'yolov4-tiny.weights')
        self.model = cv2.dnn_DetectionModel(self.net)
        self.model.setInputParams(size=(320,320), scale=1/255)
        self.classes = [] 
        self.x = 0
        self.y = 0
        self.h =0
        self.w = 0
        self.score = 0.5
        self.class_name = ''
        self.open_getname()
        
        
    def open_getname(self):
        with open('classes.txt') as file_obj:
            for class_name in file_obj.readlines():     
                class_name = class_name.strip()   
                self.classes.append(class_name)
                
    def detect_image(self, image):
        
        classids, scores, bboxes = self.model.detect(image, 0.5, 0.3)
        
        for class_id, self.score, bbox in zip(classids, scores, bboxes):  
            self.x, self.y, self.w, self.h = bbox
            self.class_name = self.classes[class_id]
            
            cv2.rectangle(image, (self.x,self.y), (self.x+self.w,self.y+self.h), (255,255,0), 2)
        
            cv2.putText(image, self.class_name, (self.x,self.y+self.h+20), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
        
            cv2.putText(image, str(int(self.score*100))+'%', (self.x,self.y-5), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,255), 2)
        
        return image
    
    def garbage_run(self, img):
 
        img = cv.resize(img, (640, 480))
        try: img = self.detect_image(img)  # 获取识别消息
        except Exception: print("get_pos NoneType")
        return img