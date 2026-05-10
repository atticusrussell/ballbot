#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from dofbot_pro_apriltag.media_library import *
from collections import defaultdict
import threading
from std_msgs.msg import Float32, Int8, Bool
from dofbot_pro_interface.msg import *
from time import sleep, time
from Arm_Lib import Arm_Device
# 全局初始化
ORDER = defaultdict(list)
order_lock = threading.Lock()

class DetectGesture(Node):
    def __init__(self):
        super().__init__('detect_gesture_node')
        # self.media_ros = Media_ROS()
        self.hand_detector = HandDetector()
        self.pub_gesture = True
        self.Arm = Arm_Device()
        # ROS2 订阅者
        qos = QoSProfile(depth=10)
        self.img_sub = self.create_subscription(ImageMsg, "/image_data",self.image_sub_callback, qos)
        self.grasp_sub= self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback, qos)
        # ROS2 发布者
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", qos)
        self.pub_targetID = self.create_publisher(Int8, "TargetId", qos)
        self.pTime = self.cTime = 0
        self.cnt = 0
        self.last_sum = 0
        self.detect_gesture = Int8()
        self.pTime = 0
        # 初始化其他变量
        self.detect_gesture_joints = [90.0, 150.0, 12.0, 20.0, 90.0, 30.0]
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.Arm.Arm_serial_servo_write6_array(self.detect_gesture_joints,2000)
        
    def GraspStatusCallback(self,msg):
        if msg.data == True:
            print("Publish the next gesture")
            self.pub_gesture = True    
            self.cnt = 0
        		
    def process(self, frame):
        #frame = cv.flip(frame, 1)
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0:
            threading.Thread(target=self.Gesture_Detect_threading, args=(lmList,bbox)).start()
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        #self.media_ros.pub_imgMsg(frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            rospy.signal_shutdown("exit")
        cv.imshow('frame', frame)

    def Gesture_Detect_threading(self, lmList,bbox):
        fingers = self.hand_detector.fingersUp(lmList)
        #print("sum of fingers: ",sum(fingers))
        self.last_sum = sum(fingers)
        print(self.pub_gesture)
        if sum(fingers) == self.last_sum:
            print("---------------------------")
            self.cnt = self.cnt + 1
            print("cnt: ",self.cnt)
            if self.cnt==40 and self.pub_gesture == True:
                print("sum of fingers: ",self.last_sum)
                self.pub_gesture = False
                self.detect_gesture.data = self.last_sum   
                self.pub_targetID.publish(self.detect_gesture)
                self.last_sum = 0
                self.cnt = 0
      
        
    def image_sub_callback(self,msg):
        image = np.ndarray(shape=(msg.height, msg.width, msg.channels), dtype=np.uint8, buffer=msg.data) # 将自定义图像消息转化为图像
        self.img[:,:,0],self.img[:,:,1],self.img[:,:,2] = image[:,:,2],image[:,:,1],image[:,:,0] # 将rgb 转化为opencv的bgr顺序
        frame = self.img.copy()
        self.process(frame)
        
def main(args=None):
    rclpy.init(args=args)
    detect_gesture1 = DetectGesture()
    try:    
        rclpy.spin(detect_gesture1)
    except KeyboardInterrupt:
        pass
    finally:
        detect_gesture1.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()