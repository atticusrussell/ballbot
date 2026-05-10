import time
import smbus
import Arm_Lib
from Speech_Lib import Speech
import os
mySpeech = Speech()
Arm = Arm_Lib.Arm_Device()
time.sleep(.1)
os.system("mpg123 /home/jetson/speech_music/0.mp3")

servo_1 = 90
def main():
    Arm.Arm_Button_Mode(1)
    time.sleep(.5)
    mySpeech.void_write(29)
    while True:
        global servo_1
        result = mySpeech.speech_read()
        if result == 55:
            time.sleep(.1)
            Arm.Arm_Action_Study()
            time.sleep(.5)
            num = Arm.Arm_Read_Action_Num()
            print(num)
            time.sleep(.5)
            if num < 20:
                mySpeech.void_write(55)
            elif num >= 20:  
                mySpeech.void_write(56)
        elif result == 56:
            mySpeech.void_write(57)
            Arm.Arm_Button_Mode(0)
            
            
        elif result == 57:
            mySpeech.void_write(58)
            Arm.Arm_Action_Mode(1)
            
        elif result == 58:
            mySpeech.void_write(59)
            Arm.Arm_Clear_Action()


        time.sleep(0.5)
        #print(" END OF LINE! ")
try :
    main()
except KeyboardInterrupt:
    # 释放Arm对象
    del Arm
    del speech
    print(" Program closed! ")
    pass