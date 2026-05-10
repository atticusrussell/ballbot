import time
import threading
import sys
import serial
import os


# V0.0.3
class Speech(object):
    def __init__(self,path = '~/speech_music/', com="/dev/myspeech"):
        # com="/dev/ttyUSB0"
        self.ser = serial.Serial(com, 115200)
        #self.ser = serial.Serial("/dev/ttyUSB0", 115200)
        self.language = 'cn' #默认是中文
        self.path = path
        
        if self.ser.isOpen():
            print("Speech Serial Opened! Baudrate=115200")
        else:
            print("Speech Serial Open Failed!")

    def __del__(self):
        self.ser.close()
        print("speech serial Close!")
        
        
    #播放音频
    def play_voide(self,viocenum): 
        if self.language == 'cn':
            stryy = self.path+'answer'+str(viocenum)+'.mp3'
            try:
                os.system("mpg123 "+stryy+"< /dev/null > /dev/null 2>1 &")
                print("Finish play.")
            except:
                print("音频找不到")
        else:
            stryy = self.path+str(viocenum)+'_en.mp3'
            try:
                os.system("mpg123 "+stryy+"< /dev/null > /dev/null 2>1 &")
            except:
                print("Audio not found!")

    # 选择播报语句
    def void_write(self, void_data): #上层控制音频的播放接口
        if self.language == 'cn':
            stryy = self.path+'answer'+str(void_data)+'.mp3'
            try:
                os.system("mpg123 "+stryy+"< /dev/null > /dev/null 2>1 &")
            except:
                print("音频找不到")
        else:
            stryy = self.path+'answer'+str(void_data)+'_en.mp3'
            try:
                os.system("mpg123 "+stryy+"< /dev/null > /dev/null 2>1 &")
            except:
                print("Audio not found!")



    # 读取识别的语音
    def speech_read(self):
        count = self.ser.inWaiting()
        #print("data:"+str(count))
        if count:
            speech_data = self.ser.read(count)
            hex_data = speech_data.hex()
            
                # 提取 '00' 和 '03' 部分
            byte1 = hex_data[4:6]  # 提取 '00'
            byte2 = hex_data[6:8]  # 提取 '03'
                # 将十六进制转换为整数
            value1 = int(byte1, 16)
            value2 = int(byte2, 16)
            
            if value1 != 0 and value1 <4: #是中文唤醒
                self.language = 'cn'
                self.play_voide(0)#播报中文音频
                return 0
            elif value1 != 0 and value1 >=4:
                self.language = 'en'
                self.play_voide(0)#播报英文音频
                return 0
                
            # else: #这个用上位机控制吧，不要库发送了
            #     self.play_voide(value2)#播报音频
                
            self.ser.flushInput()
            time.sleep(0.005)
            return value2
        else:
            return 999
