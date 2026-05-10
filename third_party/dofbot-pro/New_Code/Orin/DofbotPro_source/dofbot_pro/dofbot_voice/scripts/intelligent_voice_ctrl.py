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
        print("result: ",result)
        #鼓掌
        if result == 45:
            mySpeech.void_write(45) 
            s_time = 500
            Arm.Arm_serial_servo_write(6, 180, s_time)
            time.sleep(s_time/1000)
            Arm.Arm_serial_servo_write(6, 90, s_time)
            time.sleep(s_time/1000)
            Arm.Arm_serial_servo_write(6, 180, s_time)
            time.sleep(s_time/1000)
            Arm.Arm_serial_servo_write(6, 90, s_time)
            time.sleep(s_time/1000)
             
        elif result == 46:
            mySpeech.void_write(45) 
            s_time = 800
            Arm.Arm_serial_servo_write(4, 30, s_time)
            time.sleep(s_time/1000)
            Arm.Arm_serial_servo_write(4, 0, s_time)
            time.sleep(s_time/1000)
            Arm.Arm_serial_servo_write(4, 30, s_time)
            time.sleep(s_time/1000)
            Arm.Arm_serial_servo_write(4, 0, s_time)
            time.sleep(s_time/1000)
             
        elif result == 47:
            mySpeech.void_write(45) 
            Arm.Arm_serial_servo_write6(90, 90, 0, 180, 90, 180, 1000)
            time.sleep(1.5)
            Arm.Arm_serial_servo_write6(90, 164, 18, 0, 90, 90, 1000)
            time.sleep(1) 
        elif result == 48:
            mySpeech.void_write(45) 
            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 800)
            time.sleep(.1)
            Arm.Arm_serial_servo_write6(90, 0, 180, 0, 90, 30, 1000)
            time.sleep(1)
        elif result == 49:
            mySpeech.void_write(45) 
            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 800)
            time.sleep(1)
        elif result == 50:
            mySpeech.void_write(45) 
            s_time = 800
            Arm.Arm_serial_servo_write6(0, 90, 90, 90, 90, 90, 800)
            time.sleep(s_time/1000)
            Arm.Arm_serial_servo_write6(180, 90, 90, 90, 90, 90, 800)
            time.sleep(s_time/1000)
            Arm.Arm_serial_servo_write6(0, 90, 90, 90, 90, 90, 800)
            time.sleep(s_time/1000)
            Arm.Arm_serial_servo_write6(180, 90, 90, 90, 90, 90, 800)
            time.sleep(s_time/1000)
            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 800)

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