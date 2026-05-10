# !/usr/bin/env python3
# coding: utf-8
from Arm_Lib import Arm_Device
import time


class Robot_Controller:
    def __init__(self):

        self.Arm = Arm_Device()

        self.gripper_clamp = 135
        self.gripper_release = 30
        self.joint5 = 90

        # 颜色的位置
        self.P_BLUE = [150, 30, 70, 5, 60, self.gripper_clamp]
        self.P_GREEN = [180, 35, 60, 0, 90, self.gripper_clamp]
        self.P_RED = [28, 30, 70, 2, 120, self.gripper_clamp]
        self.P_YELLOW = [0, 43, 48, 6, 90, self.gripper_clamp]

        # 数字的位置
        self.P_NUM_1 = [150, 30, 70, 5, 60, self.gripper_clamp]
        self.P_NUM_2 = [180, 35, 60, 0, 90, self.gripper_clamp]
        self.P_NUM_3 = [28, 30, 70, 2, 120, self.gripper_clamp]
        self.P_NUM_4 = [0, 43, 48, 6, 90, self.gripper_clamp]
        
        # 中心十字架的位置
        self.P_CENTER = [90, 35, 65, 0, 90, self.gripper_clamp]
        self.P_CENTER_2 = [90, 50, 55, 0, 90, self.gripper_clamp]
        self.P_CENTER_3 = [90, 70, 40, 2, 90, self.gripper_clamp]
        self.P_CENTER_4 = [90, 90, 25, 10, 90, self.gripper_clamp]

        # 堆叠到中心十字架的位置
        self.P_CENTER_HEAP_L1 = [90, 35, 65, 0, 90, self.gripper_clamp]
        self.P_CENTER_HEAP_L2 = [90, 50, 55, 0, 90, self.gripper_clamp]
        self.P_CENTER_HEAP_L3 = [90, 70, 40, 2, 90, self.gripper_clamp]
        self.P_CENTER_HEAP_L4 = [90, 90, 28, 5, 90, self.gripper_clamp]

        # 颜色/标签堆叠的位置
        self.P_HEAP_1 = [180, 35, 65, 0, 90, self.gripper_clamp]
        self.P_HEAP_2 = [180, 50, 55, 0, 90, self.gripper_clamp]
        self.P_HEAP_3 = [180, 70, 40, 2, 90, self.gripper_clamp]
        self.P_HEAP_4 = [180, 90, 28, 5, 90, self.gripper_clamp]

        # 推倒中间积木的位置
        self.P_OVER_1 = [90, 80, 10, 10, 90, self.gripper_clamp+30]
        self.P_OVER_2 = [90, 80, 10, 60, 90, self.gripper_clamp+30]

        # 垃圾的位置
        self.P_RECYCLABLE_WASTE = [140, 20, 90, 3, 50, self.gripper_clamp]
        self.P_KITCHEN_WASTE = [165, 38, 60, 2, 70, self.gripper_clamp]
        self.P_HAZARDOUS_WASTE = [38, 20, 90, 2, 125, self.gripper_clamp]
        self.P_OTHER_WASTE = [12, 38, 60, 0, 100, self.gripper_clamp]

        # 自定义姿态的位置
        self.P_TOP = [90, 80, 50, 50, 90, self.gripper_clamp]
        self.P_LOOK_AT = [90, 164, 18, 0, 90, self.gripper_release]
        self.P_LOOK_MAP = [90, 106, 0, 0, 90, self.gripper_release]
        self.P_LOOK_FRONT = [90, 135, 20, 25, 90, self.gripper_release]
        self.P_LOOK_UP = [90, 90, 90, 90, 90, 180]
        self.P_FINGER_START = [90, 131, 52, 0, 90, 180]
        self.P_SNAKE_INIT = [90, 135, 0, 45, 0, 180]
        self.P_POSE_INIT = [90, 164, 18, 5, 90, self.gripper_release]

        # 自定义动作组
        self.P_ACTION_1 = [90, 90, 90, 90, 90, 180]
        self.P_ACTION_2 = [90, 90, 0, 180, 90, 180]
        self.P_ACTION_3 = [90, 131, 52, 0, 90, 180]
        self.P_ACTION_4 = [90, 0, 180, 20, 90, 30]
        self.P_ACTION_5 = [90, 0, 90, 180, 90, 0]

        self.P_RIGHT_UP = [0, 90, 0, 180, 90, 180]
        self.P_LEFT_UP = [180, 90, 0, 180, 90, 180]
        self.P_HANDS_UP = [90, 90, 90, 90, 90, 30]


    def arm_move_1(self, s_id, angle, time_ms, delay=False):
        self.Arm.Arm_serial_servo_write(s_id, angle, int(time_ms))
        if delay and time_ms > 0:
            time.sleep(time_ms/1000.0)
    
    def arm_move_5(self, p, time_ms, delay=False):
        for i in range(5):
            s_id = i + 1
            self.Arm.Arm_serial_servo_write(s_id, p[i], int(time_ms))
            time.sleep(.002)
        if delay and time_ms > 0:
            time.sleep(time_ms/1000.0)

    def arm_move(self, p, time_ms, delay=False):
        for i in range(5):
            s_id = i + 1
            self.Arm.Arm_serial_servo_write(s_id, p[i], int(time_ms))
            time.sleep(.002)
        if delay and time_ms > 0:
            time.sleep(time_ms/1000.0)

    def arm_move_6(self, joint6_array, time_ms, delay=False):
        if len(joint6_array) == 6:
            self.Arm.Arm_serial_servo_write6_array(joint6_array, time_ms)
            if delay and time_ms > 0:
                time.sleep(time_ms/1000.0)

    

    def arm_clamp_block(self, state, time_ms=500, delay=False):
        if state:
            self.Arm.Arm_serial_servo_write(6, self.gripper_clamp, int(time_ms))
        else:
            self.Arm.Arm_serial_servo_write(6, self.gripper_release, int(time_ms))
        if delay and time_ms > 0:
            time.sleep(time_ms/1000.0)


    def ctrl_gripper(self, value, time_ms, delay=False):
        if value > 180: value = 180
        if value < 0: value = 0
        self.Arm.Arm_serial_servo_write(6, int(value), int(time_ms))
        if delay and time_ms > 0:
            time.sleep(time_ms/1000.0)

    def get_gripper_value(self, state):
        if state == 0:
            return self.gripper_release
        return self.gripper_clamp

    # 机械臂看地图
    def move_look_map(self, time_ms=1000, delay=False):
        self.Arm.Arm_serial_servo_write6_array(self.P_LOOK_MAP, time_ms)
        if delay:
            time.sleep(time_ms/1000.0)
    
    # 机械臂初始状态，看前面
    def move_init_pose(self, time_ms=1000, delay=False):
        self.Arm.Arm_serial_servo_write6_array(self.P_LOOK_AT, time_ms)
        if delay:
            time.sleep(time_ms/1000.0)
    
    # 机械臂看前面
    def move_look_front(self, time_ms=1000, delay=False):
        self.Arm.Arm_serial_servo_write6_array(self.P_LOOK_FRONT, time_ms)
        if delay:
            time.sleep(time_ms/1000.0)

    # 控制蜂鸣器
    def Arm_Buzzer_On(self, delay=0xff):
        self.Arm.Arm_Buzzer_On(delay)
