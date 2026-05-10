#!/usr/bin/env python3
# encoding: utf-8
import os
import rospy
import threading
import time
from sensor_msgs.msg import Joy
from dofbot_utils.robot_controller import Robot_Controller


class Dofbot_Joystick:
    def __init__(self, debug=False):
        rospy.on_shutdown(self.cancel)
        self.debug = debug
        self.robot = Robot_Controller()
        self.ctrl_step = 0
        self.press_state = 0
        self.joins_active = [False for i in range(6)]
        self.arm_joints = [False for i in range(6)]
        self.ANGLE_MIN = [0, 0, 0, 0, 0, 30]
        self.ANGLE_MAX = [180, 180, 180, 180, 180, 180]
        self.init_pose()
        time.sleep(1)
        self.sub_Joy = rospy.Subscriber('joy', Joy, self.joystick_callback)

        self.Joy_Button_Index = {
            # BUTTON
            "KEY_A": 0,
            "KEY_B": 1,
            "KEY_X": 3,
            "KEY_Y": 4,
            "KEY_L1": 6,
            "KEY_R1": 7,
            "KEY_L2": 8,
            "KEY_R2": 9,
            "KEY_SELECT": 10,
            "KEY_START": 11,
            "KEY_RK1": 12,
            "KEY_RK2": 13
        }

        self.Joy_Axis_Index = {
            # AXIS
            "RK1_LEFT_RIGHT": 0,
            "RK1_UP_DOWN": 1,
            "RK2_LEFT_RIGHT": 2,
            "RK2_UP_DOWN": 3,
            "AXIS_R2": 4,
            "AXIS_L2": 5,
            "WSAD_LEFT_RIGHT": 6,
            "WSAD_UP_DOWN": 7
        }

    # 初始化数值
    def reset_value(self):
        self.ctrl_step = 2
        self.press_state = 0
        for i in range(6):
            self.joins_active[i] = False 
            self.arm_joints[i] = self.robot.P_LOOK_AT[i]

    # 机器人初始状态的位姿
    def init_pose(self):
        self.reset_value()
        if self.debug:
            print("arm_joints:", self.arm_joints)
        self.robot.arm_move_6(self.arm_joints, 1000)

    # 控制夹爪，state=0为松开，state=1为夹紧积木
    def ctrl_gripper(self, state):
        if state:
            self.joins_active[5] = False
            self.arm_joints[5] = self.robot.get_gripper_value(1)
        else:
            self.joins_active[5] = False
            self.arm_joints[5] = self.robot.get_gripper_value(0)
        self.robot.arm_move_1(6, self.arm_joints[5], 500)


    # 控制机器人运动
    def ctrl_machine(self, id, step):
        while True:
            if self.joins_active[id-1]:
                self.arm_joints[id-1] += step
                if self.arm_joints[id-1] > self.ANGLE_MAX[id-1]:
                    self.arm_joints[id-1] = self.ANGLE_MAX[id-1]
                elif self.arm_joints[id-1] < self.ANGLE_MIN[id-1]:
                    self.arm_joints[id-1] = self.ANGLE_MIN[id-1]
                rospy.loginfo("joints:%d = %d" % (id, self.arm_joints[id-1]))
                self.robot.arm_move_1(id, self.arm_joints[id-1], 500)
                time.sleep(0.05)
            else:
                break

    # 根据舵机ID和一步的差值来控制舵机
    def update_joints(self, id, step):
        if step == 0:
            self.joins_active[id-1] = False
            return
        if self.joins_active[id-1]:
            return
        self.joins_active[id-1] = True
        arm_thread = threading.Thread(target=self.ctrl_machine, args=(id, step))
        arm_thread.setDaemon(True)
        arm_thread.start()

    # 根据手柄的按键状态，获取当前关节对应一步的差值
    def key_to_step(self, joy_data, key_increase, key_decrease):
        value_inc = int(joy_data.buttons[self.Joy_Button_Index.get(key_increase)])
        value_dec = int(joy_data.buttons[self.Joy_Button_Index.get(key_decrease)])
        result = 0
        if value_inc != 0:
            result = self.ctrl_step
        elif value_dec != 0:
            result = -self.ctrl_step
        return result
    
    # 根据手柄的轴状态，获取当前关节对应一步的差值
    def axis_to_step(self, joy_data, axis):
        result = self.ctrl_step * int(joy_data.axes[self.Joy_Axis_Index.get(axis)])
        return result
    
    # 根据手柄的按键状态，更新一步的差距
    def update_step(self, joy_data, key_increase, key_decrease):
        step = self.ctrl_step
        if joy_data.buttons[self.Joy_Button_Index.get(key_increase)]:
            if self.press_state == 0:
                self.press_state = 1
                step = self.ctrl_step + 1
                if step > 3:
                    step = 3
            rospy.loginfo("step:%d" % step)
        else:
            if self.press_state == 1:
                self.press_state = 0
        if joy_data.buttons[self.Joy_Button_Index.get(key_decrease)]:
            if self.press_state == 0:
                self.press_state = -1
                step = self.ctrl_step - 1
                if step < 1:
                    step = 1
            rospy.loginfo("step:%d" % step)
        else:
            if self.press_state == -1:
                self.press_state = 0
        self.ctrl_step = step
        return self.ctrl_step

    # 手柄数据更新的回调函数
    def joystick_callback(self, joy_data):
        if not isinstance(joy_data, Joy):
            return
        if self.debug:
            print("joy_data:", joy_data.buttons, joy_data.axes)
        self.update_step(joy_data, "KEY_L1", "KEY_L2")
        self.update_joints(1, self.axis_to_step(joy_data, "RK1_LEFT_RIGHT"))
        self.update_joints(2, self.axis_to_step(joy_data, "RK1_UP_DOWN"))
        self.update_joints(3, self.axis_to_step(joy_data, "WSAD_UP_DOWN"))
        self.update_joints(4, self.axis_to_step(joy_data, "RK2_UP_DOWN"))
        self.update_joints(5, -self.axis_to_step(joy_data, "RK2_LEFT_RIGHT"))
        self.update_joints(6, self.key_to_step(joy_data, "KEY_Y", "KEY_A"))
        if joy_data.buttons[self.Joy_Button_Index.get("KEY_R1")]:
            self.ctrl_gripper(1)
        if joy_data.buttons[self.Joy_Button_Index.get("KEY_R2")]:
            self.ctrl_gripper(0)
        if joy_data.buttons[self.Joy_Button_Index.get("KEY_SELECT")]:
            self.init_pose()
        if joy_data.buttons[self.Joy_Button_Index.get("KEY_START")]:
            self.robot.Arm_Buzzer_On(2)

    def cancel(self):
        self.sub_Joy.unregister()
        


if __name__ == '__main__':
    rospy.init_node('joy_ctrl')
    joy = Dofbot_Joystick()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('exception')
