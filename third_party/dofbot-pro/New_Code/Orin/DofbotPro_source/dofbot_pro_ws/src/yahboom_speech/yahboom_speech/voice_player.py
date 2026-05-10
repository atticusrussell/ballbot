import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
import pandas as pd

file_path = "/home/jetson/speech_ID.xlsx"
csv_file = "/home/jetson/speech_ID.csv"
dfs = pd.read_excel(file_path, nrows=121,sheet_name='cn',engine='openpyxl')
dfs.to_csv(csv_file, index=False)
df_csv = pd.read_csv(csv_file)

from Speech_Lib import Speech
mySpeech = Speech()


class VoicePlayer(Node):
    def __init__(self):
        super().__init__('Player_Node')
        self.sub_playID = self.create_subscription(Int8,'player_id',self.playercallback,1000)
        
    def playercallback(self,msg):
        ID_value = msg.data
        print("ID_value: ",ID_value)
        if ID_value!=0:
        	'''res = df_csv[df_csv['M2C'] == ID_value]['C2M']
        	play_id = res.iloc[0]'''
        	play_id = ID_value
        	print("play_id = ",play_id)
        	mySpeech.void_write(int(play_id))

def main(args=None):
    rclpy.init(args=args)
    voice_player = VoicePlayer()
    try:    
        rclpy.spin(voice_player)
    except KeyboardInterrupt:
        pass
    finally:
        voice_player.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()     

    
