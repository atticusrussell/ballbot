import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
import pandas as pd

file_path = "/home/jetson/speech_ID.xlsx"
csv_file = "/home/jetson/speech_ID.csv"


from Speech_Lib import Speech
mySpeech = Speech()


class VoiceReconize(Node):
    def __init__(self):
        super().__init__('VoiceReconize_Node')
        self.pub_res = self.create_publisher(Int8,'voice_result',1)
        self.voice_command = Int8()
        
    def pub_data(self):
        while True:
            result = mySpeech.speech_read()
            # print("resulr",result)
            if result!=999 and result!=0:
                self.voice_command.data = result
                self.pub_res.publish(self.voice_command) 
                print("result: ",result)

def main(args=None):
    rclpy.init(args=args)
    voice_reconize = VoiceReconize()
    voice_reconize.pub_data()
    try:    
        rclpy.spin(voice_reconize)
    except KeyboardInterrupt:
        pass
    finally:
        voice_reconize.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()    

    
