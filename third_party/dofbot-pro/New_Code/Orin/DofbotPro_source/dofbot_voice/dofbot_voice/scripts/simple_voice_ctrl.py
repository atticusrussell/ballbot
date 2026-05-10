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
    while True:
        global servo_1
        result = mySpeech.speech_read()
        #print(result)
        if result == 11:
            Arm.Arm_RGB_set(50, 0, 0) #RGB亮红灯
            mySpeech.void_write(11) 
        elif result == 12:
            Arm.Arm_RGB_set(0, 50, 0) #RGB亮绿灯
            mySpeech.void_write(12) 
        elif result == 13:
            Arm.Arm_RGB_set(0, 0, 50) #RGB亮蓝灯
            mySpeech.void_write(13)  
        elif result == 38:
            Arm.Arm_Buzzer_On(3)
            time.sleep(.5)
            mySpeech.void_write(38)
        elif result == 39:
            Arm.Arm_serial_servo_write(2, 90, 1000)
            time.sleep(.001)
            Arm.Arm_serial_servo_write(3, 90, 1000)
            time.sleep(.001)
            Arm.Arm_serial_servo_write(4, 90, 1000)
            time.sleep(1)
            mySpeech.void_write(39)
        elif result == 40:
            Arm.Arm_serial_servo_write(2, 50, 1000)
            time.sleep(.001)
            Arm.Arm_serial_servo_write(3, 50, 1000)
            time.sleep(.001)
            time.sleep(1)
            mySpeech.void_write(40)
        elif result == 41:
            servo_1 += 30
            if servo_1 >= 120:
                servo_1 = 120
            Arm.Arm_serial_servo_write(1, servo_1, 500)
            time.sleep(1)
            mySpeech.void_write(41)
        elif result == 42:
            servo_1 -= 60
            if servo_1 <= 60:
                servo_1 = 60
            Arm.Arm_serial_servo_write(1, servo_1, 500)
            time.sleep(1)
            mySpeech.void_write(42)
        elif result == 43:
            Arm.Arm_serial_servo_write(6, 180, 500)
            time.sleep(1)
            mySpeech.void_write(43)
        elif result == 44:
            Arm.Arm_serial_servo_write(6, 90, 500)
            time.sleep(1)
            mySpeech.void_write(44)

        time.sleep(0.5)
        #print(" END OF LINE! ")
try :
    main()
except KeyboardInterrupt:
    # 释放Arm对象
    del Arm
    del mySpeech
    print(" Program closed! ")
    pass