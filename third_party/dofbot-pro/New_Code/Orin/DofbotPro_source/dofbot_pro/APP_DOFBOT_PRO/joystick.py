#!/usr/bin/env python3
# coding=utf-8
import os, struct, sys
import time
import threading
from dofbot_utils.robot_controller import Robot_Controller


# V1.0.2
class Joystick(object):

    def __init__(self, robot, js_id=0, debug=False):
        self.__debug = debug
        self.__js_id = int(js_id)
        self.__js_isOpen = False
        self.__ignore_count = 24
        self.robot = robot
        self.ctrl_step = 0
        self.press_state = 0
        self.joins_active = [False for i in range(6)]
        self.arm_joints = [False for i in range(6)]

        self.STATE_OK = 0
        self.STATE_NO_OPEN = 1
        self.STATE_DISCONNECT = 2
        self.STATE_KEY_BREAK = 3

        # Find the joystick device.
        print('Joystick Available devices:')
        # Shows the joystick list of the Controler, for example: /dev/input/js0
        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                print('    /dev/input/%s' % (fn))

        # Open the joystick device.
        try:
            js = '/dev/input/js' + str(self.__js_id)
            self.__jsdev = open(js, 'rb')
            self.__js_isOpen = True
            print('---Opening %s Succeeded---' % js)
        except:
            self.__js_isOpen = False
            print('---Failed To Open %s---' % js)

        self.ANGLE_MIN = [0, 0, 0, 0, 0, 30]
        self.ANGLE_MAX = [180, 180, 180, 180, 180, 180]
        self.reset_value()
        
        # Defining Functional List
        # Red LED Mode
        self.__function_names = {
            # BUTTON FUNCTION
            0x0100: 'A',
            0x0101: 'B',
            0x0103: 'X',
            0x0104: 'Y',
            0x0106: 'L1',
            0x0107: 'R1',
            0x0108: 'L2_1',
            0x0109: 'R2_1',
            0x010A: 'SELECT',
            0x010B: 'START',
            0x010D: 'BTN_RK1',
            0x010E: 'BTN_RK2',

            # AXIS FUNCTION
            0x0200: 'RK1_LEFT_RIGHT',
            0x0201: 'RK1_UP_DOWN',
            0x0202: 'RK2_LEFT_RIGHT',
            0x0203: 'RK2_UP_DOWN',
            0x0204: 'R2',
            0x0205: 'L2',
            0x0206: 'WSAD_LEFT_RIGHT',
            0x0207: 'WSAD_UP_DOWN',
        }

    def __del__(self):
        if self.__js_isOpen:
            self.__jsdev.close()
        if self.__debug:
            print("\n---Joystick DEL---\n")

    # Return joystick state
    def is_Opened(self):
        return self.__js_isOpen
    
    # Control robot
    def __data_processing(self, name, value):
        
        if name=="RK1_LEFT_RIGHT":
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            self.update_joints(1, self.value_to_step(value))
        
        elif name == 'RK1_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            self.update_joints(2, self.value_to_step(value))

        elif name == 'RK2_LEFT_RIGHT':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            self.update_joints(5, -self.value_to_step(value))

            
        elif name == 'RK2_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            self.update_joints(4, self.value_to_step(value))

        elif name == 'A':
            if self.__debug:
                print (name, ":", value)
            self.update_joints(6, -self.value_to_step(value))

        elif name == 'B':
            if self.__debug:
                print (name, ":", value)

        elif name == 'X':
            if self.__debug:
                print (name, ":", value)

        elif name == 'Y':
            if self.__debug:
                print (name, ":", value)
            self.update_joints(6, self.value_to_step(value))

        elif name == 'L1':
            if self.__debug:
                print (name, ":", value)
            if value == 1:
                self.ctrl_step = self.ctrl_step + 1
                if self.ctrl_step >= 3:
                    self.ctrl_step = 3

        elif name == 'R1':
            if self.__debug:
                print (name, ":", value)
            if value == 1:
                self.ctrl_gripper(1)
        elif name == 'SELECT':
            if self.__debug:
                print (name, ":", value)
            if value == 1:
                self.init_pose()

        elif name == 'START':
            if self.__debug:
                print (name, ":", value)
            if value == 1:
                self.robot.Arm_Buzzer_On(2)
        
        elif name == 'MODE':
            if self.__debug:
                print (name, ":", value)
        elif name == 'BTN_RK1':
            if self.__debug:
                print (name, ":", value)

        elif name == 'BTN_RK2':
            if self.__debug:
                print (name, ":", value)
        
        elif name == "L2":
            value = ((value/32767)+1)/2
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            if value == 1:
                self.ctrl_step = self.ctrl_step - 1
                if self.ctrl_step <= 1:
                    self.ctrl_step = 1

        elif name == "R2":
            value = ((value/32767)+1)/2
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            if value == 1:
                self.ctrl_gripper(0)

            
        elif name == 'WSAD_LEFT_RIGHT':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))

        elif name == 'WSAD_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            self.update_joints(3, self.value_to_step(value))
        else:
            pass

    # Handles events for joystick
    def joystick_handle(self):
        if not self.__js_isOpen:
            # if self.__debug:
            #     print('Failed To Open Joystick')
            return self.STATE_NO_OPEN
        try:
            evbuf = self.__jsdev.read(8)
            if evbuf:
                timestamp, value, type, number = struct.unpack('IhBB', evbuf)
                func = type << 8 | number
                name = self.__function_names.get(func)
                # print("evbuf:", timestamp, value, type, number)
                # if self.__debug:
                #     print("func:0x%04X, %s, %d" % (func, name, value))
                if name != None:
                    self.__data_processing(name, value)
                else:
                    if self.__ignore_count > 0:
                        self.__ignore_count = self.__ignore_count - 1
                    if self.__debug and self.__ignore_count == 0:
                        print("Key Value Invalid")
            return self.STATE_OK
        except KeyboardInterrupt:
            if self.__debug:
                print('Key Break Joystick')
            return self.STATE_KEY_BREAK
        except:
            self.__js_isOpen = False
            print('---Joystick Disconnected---')
            return self.STATE_DISCONNECT

    # reconnect Joystick
    def reconnect(self):
        try:
            js = '/dev/input/js' + str(self.__js_id)
            self.__jsdev = open(js, 'rb')
            self.__js_isOpen = True
            self.__ignore_count = 24
            print('---Opening %s Succeeded---' % js)
            return True
        except:
            self.__js_isOpen = False
            # if self.__debug:
            #     print('Failed To Open %s' % js)
            return False

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
        if self.__debug:
            print("arm_joints:", self.arm_joints)
        self.robot.arm_move_6(self.arm_joints, 1000)

    # 控制夹爪，state=0为松开，state=1为夹紧
    def ctrl_gripper(self, state):
        if state:
            self.joins_active[5] = False
            self.arm_joints[5] = self.robot.get_gripper_value(1)
        else:
            self.joins_active[5] = False
            self.arm_joints[5] = self.robot.get_gripper_value(0)
        if self.__debug:
            print("arm6:", self.arm_joints[5])
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
                if self.__debug:
                    print("Joy:", id, self.arm_joints[id-1])
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
        arm_thread.Daemon=True
        arm_thread.start()


    def value_to_step(self, value):
        if value == 1:
            return self.ctrl_step
        elif value == -1:
            return -self.ctrl_step
        else:
            return 0


if __name__ == '__main__':
    g_debug = False
    if len(sys.argv) > 1:
        if str(sys.argv[1]) == "debug":
            g_debug = True
    print("debug=", g_debug)

    g_robot = Robot_Controller()
    g_robot.move_init_pose()
    js = Joystick(g_robot, debug=g_debug)
    try:
        while True:
            state = js.joystick_handle()
            if state != js.STATE_OK:
                if state == js.STATE_KEY_BREAK:
                    break
                time.sleep(1)
                js.reconnect()
    except KeyboardInterrupt:
        pass
    del js
