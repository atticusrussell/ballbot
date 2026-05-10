import time
import smbus
import Arm_Lib
from Speech_Lib import Speech
import os
from dofbot_utils.robot_controller import Robot_Controller
robot = Robot_Controller()
mySpeech = Speech()
Arm = Arm_Lib.Arm_Device()
time.sleep(.1)
os.system("mpg123 /home/jetson/speech_music/0.mp3")

p_mould = robot.P_LOOK_AT
p_top = robot.P_TOP
p_Brown = robot.P_CENTER

p_layer_4 = robot.P_CENTER_HEAP_L4
p_layer_3 = robot.P_CENTER_HEAP_L3
p_layer_2 = robot.P_CENTER_HEAP_L2
p_layer_1 = robot.P_CENTER_HEAP_L1

p_move_layer_4 = robot.P_CENTER_4
p_move_layer_3 = robot.P_CENTER_3
p_move_layer_2 = robot.P_CENTER_2
p_move_layer_2 = robot.P_CENTER

p_Yellow = robot.P_YELLOW
p_Red = robot.P_RED

p_Green = robot.P_GREEN
p_Blue = robot.P_BLUE


#叠罗汉动作参数
def arm_clamp_block(enable):
    if enable == 0:
        Arm.Arm_serial_servo_write(6, 60, 400)
    else:
        Arm.Arm_serial_servo_write(6, 135, 400)
    time.sleep(.5)

    
# 定义移动机械臂函数,同时控制1-5号舵机运动，p=[S1,S2,S3,S4,S5]
def arm_move(p, s_time = 500):
    for i in range(5):
        id = i + 1
        if id == 5:
            time.sleep(.1)
            Arm.Arm_serial_servo_write(id, p[i], int(s_time*1.2))
        elif id == 1 :
            Arm.Arm_serial_servo_write(id, p[i], int(3*s_time/4))

        else:
            Arm.Arm_serial_servo_write(id, p[i], int(s_time))
        time.sleep(.01)
    time.sleep(s_time/1000)

def heap_up():
    # 让机械臂移动到一个准备抓取的位置
    arm_clamp_block(0)
    arm_move(p_mould, 1000)
    time.sleep(1)
    # 夹取黄色区域的方块堆叠到中间最底层的位置。
    arm_move(p_top, 1000)
    arm_move(p_Yellow, 1000)
    arm_clamp_block(1)

    arm_move(p_top, 1000)
    arm_move(p_layer_1, 1000)
    arm_clamp_block(0)

    time.sleep(.1)

    arm_move(p_mould, 1100)

    time.sleep(2)
    
    # 夹取红色区域的方块堆叠到中间第二层的位置。
    arm_move(p_top, 1000)
    arm_move(p_Red, 1000)
    arm_clamp_block(1)

    arm_move(p_top, 1000)
    arm_move(p_layer_2, 1000)
    arm_clamp_block(0)

    time.sleep(.1)

    arm_move(p_mould, 1100)

    time.sleep(2)
    
    # 夹取绿色区域的方块堆叠到中间第三层的位置。
    arm_move(p_top, 1000)
    arm_move(p_Green, 1000)
    arm_clamp_block(1)

    arm_move(p_top, 1000)
    arm_move(p_layer_3, 1000)
    arm_clamp_block(0)

    time.sleep(.1)

    arm_move(p_mould, 1100)

    time.sleep(2)
    
    # 夹取蓝色区域的方块堆叠到中间第四层的位置。
    arm_move(p_top, 1000)
    arm_move(p_Blue, 1000)

    arm_clamp_block(1)

    arm_move(p_top, 1000)
    arm_move(p_layer_4, 1000)
    arm_clamp_block(0)

    time.sleep(.1)

    arm_move(p_mould, 1100)

    time.sleep(1)


#跳舞动作参数
time_1 = 500
time_2 = 1000
time_sleep = 0.5

def dance():
    Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 500)
    time.sleep(1)
    
    Arm.Arm_serial_servo_write(2, 180-120, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(3, 120, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(4, 60, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(2, 180-135, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(3, 135, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(4, 45, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(2, 180-120, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(3, 120, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(4, 60, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(2, 90, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(3, 90, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(4, 90, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(2, 180-80, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(3, 80, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(4, 80, time_1)
    time.sleep(time_sleep)



    Arm.Arm_serial_servo_write(2, 180-60, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(3, 60, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(4, 60, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(2, 180-45, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(3, 45, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(4, 45, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(2, 90, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(3, 90, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(4, 90, time_1)
    time.sleep(.001)
    time.sleep(time_sleep)



    Arm.Arm_serial_servo_write(4, 20, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(6, 150, time_1)
    time.sleep(.001)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(4, 90, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(6, 90, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(4, 20, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(6, 150, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(4, 90, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(6, 90, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(1, 0, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(5, 0, time_1)
    time.sleep(time_sleep)



    Arm.Arm_serial_servo_write(3, 180, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(4, 0, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(6, 180, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(6, 0, time_2)
    time.sleep(time_sleep)



    Arm.Arm_serial_servo_write(6, 90, time_2)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(1, 90, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(5, 90, time_1)
    time.sleep(time_sleep)

    Arm.Arm_serial_servo_write(3, 90, time_1)
    time.sleep(.001)
    Arm.Arm_serial_servo_write(4, 90, time_1)
    time.sleep(time_sleep)


#夹方块动作参数
# 定义移动机械臂函数,同时控制1-5号舵机运动，p=[S1,S2,S3,S4,S5]
def arm_move_clamp(p, s_time = 500):
    for i in range(5):
        id = i + 1
        if id == 5:
            time.sleep(.1)
            Arm.Arm_serial_servo_write(id, p[i], int(s_time*1.2))
        else :
            Arm.Arm_serial_servo_write(id, p[i], s_time)
        time.sleep(.01)
    time.sleep(s_time/1000)

# 机械臂向上移动
def arm_move_up():
    Arm.Arm_serial_servo_write(2, 90, 1500)
    Arm.Arm_serial_servo_write(3, 90, 1500)
    Arm.Arm_serial_servo_write(4, 90, 1500)
    time.sleep(.1)

def clamp_clock():
    # 让机械臂移动到一个准备抓取的位置
    arm_clamp_block(0)
    arm_move_clamp(p_mould, 1000)
    time.sleep(1)

    # 从灰色积木块位置抓取一块积木放到黄色积木块的位置上。
    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Brown, 1000)
    arm_clamp_block(1)

    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Yellow, 1000)
    arm_clamp_block(0)

    arm_move_clamp(p_mould, 1000)

    time.sleep(2)

    # 从灰色积木块位置抓取一块积木放到红色积木块的位置上。
    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Brown, 1000)
    arm_clamp_block(1)

    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Red, 1000)
    arm_clamp_block(0)

    arm_move_up()
    arm_move_clamp(p_mould, 1100)

    time.sleep(2)

    # 从灰色积木块位置抓取一块积木放到绿色积木块的位置上。
    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Brown, 1000)
    arm_clamp_block(1)

    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Green, 1000)
    arm_clamp_block(0)

    arm_move_up()
    arm_move_clamp(p_mould, 1100)

    time.sleep(2)

    # 从灰色积木块位置抓取一块积木放到蓝色积木块的位置上。
    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Brown, 1000)
    arm_clamp_block(1)

    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Blue, 1000)
    arm_clamp_block(0)

    arm_move_up()
    arm_move_clamp(p_mould, 1100)

    time.sleep(1)


#搬运动作参数
def move_block():
    # 让机械臂移动到一个准备抓取的位置
    arm_clamp_block(0)
    arm_move_clamp(p_mould, 1000)
    time.sleep(1)

    # 搬运第四层的积木块到黄色区域
    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_move_layer_4, 1000)
    arm_clamp_block(1)

    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Yellow, 1000)
    arm_clamp_block(0)

    time.sleep(.1)

    arm_move_up()
    arm_move_clamp(p_mould, 1100)

    time.sleep(2)

    # 搬运第三层的积木块到红色区域
    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_move_layer_3, 1000)
    arm_clamp_block(1)

    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Red, 1000)
    arm_clamp_block(0)

    time.sleep(.1)

    arm_move_up()
    arm_move_clamp(p_mould, 1100)

    time.sleep(2)

    # 搬运第二层的积木块到绿色区域
    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_move_layer_2, 1000)
    arm_clamp_block(1)

    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Green, 1000)
    arm_clamp_block(0)

    time.sleep(.1)

    arm_move_up()
    arm_move_clamp(p_mould, 1100)

    time.sleep(2)

    # 搬运第一层的积木块到蓝色区域
    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_move_layer_2, 1000)
    arm_clamp_block(1)

    arm_move_clamp(p_top, 1000)
    arm_move_clamp(p_Blue, 1000)
    arm_clamp_block(0)

    time.sleep(.1)

    arm_move_up()
    arm_move_clamp(p_mould, 1100)

    time.sleep(1)


servo_1 = 90
def main():
    while True:
        global servo_1
        result = mySpeech.speech_read()
        #print(result)
        if result == 51:
            mySpeech.void_write(51) 
            heap_up()
            
        elif result == 52:
            mySpeech.void_write(52) 
            dance()
            
        elif result == 53:
            mySpeech.void_write(53)  
            clamp_clock()
        elif result == 54:
            Arm.Arm_Buzzer_On(3)
            time.sleep(.5)
            mySpeech.void_write(54)
            move_block()
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