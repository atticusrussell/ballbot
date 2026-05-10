from cv_bridge import CvBridge
import cv2
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import time
from Arm_Lib import Arm_Device


class RecordVideoNode(Node):
    def __init__(self):
        super().__init__('video_recording_node')
        self.Path_video = "/home/jetson/record_video.mp4"
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = None
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',1)
        self.rgb_bridge = CvBridge()
        self.Time_ = 0.0
        self.declare_parameter('time_', 20.0)
        self.Time_ = int(self.get_parameter('time_').get_parameter_value().double_value)
        print("Get self.target_id is ",self.Time_)
        self.record_done = False
        self.Arm = Arm_Device()
        self.beep_start = False
        self.start_time = time.time()
        self.sub_rgb = self.create_subscription(Image,"/camera/color/image_raw",self.get_RGBCallBack,100)
        print("init done.")
        

    def get_RGBCallBack(self,msg):
        
        if self.beep_start==False:
            self.start_time = time.time()
            self.beep_start = True
            self.Arm.Arm_Buzzer_On()
            time.sleep(0.5)
            self.Arm.Arm_Buzzer_Off()
            self.record_done = False
            
        if self.record_done!=True: 
            rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg,'bgr8')
            if self.out is None:
                #print("-----------")
                height, width, _ = rgb_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'x264')  # 使用MP4编码器
                self.out = cv2.VideoWriter(
                    self.Path_video, 
                    fourcc, 
                    15, 
                    (width, height)
                )
            #print("self.out: ",self.out)
            self.out.write(rgb_image)
            cv2.imshow('Video Recording', rgb_image)
            key = cv2.waitKey(1)
            elapsed = time.time() - self.start_time 
            if elapsed >= self.Time_ :
                cv2.destroyAllWindows()
                self.record_done = True
                self.out.release()
                self.Arm.Arm_Buzzer_On()
                time.sleep(0.5)
                self.Arm.Arm_Buzzer_Off()
                self.largemodel_arm_done_pub.publish(String(data='record_video_done'))
                
                
def main(args=None):
    rclpy.init(args=args)
    record_video = RecordVideoNode()
    try:    
        rclpy.spin(record_video)
    except KeyboardInterrupt:
        pass
    finally:
        record_video.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()            
            
        
            
        
        
        
        
        
        
    
    