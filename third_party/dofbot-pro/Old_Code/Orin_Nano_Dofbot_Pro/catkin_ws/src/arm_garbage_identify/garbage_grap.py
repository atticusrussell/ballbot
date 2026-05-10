#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep
from dofbot_utils.robot_controller import Robot_Controller


class garbage_grap_move:
    def __init__(self):
        # set move status
        # 设置移动状态
        self.move_status = True
        self.arm = Arm_Lib.Arm_Device()
        self.robot = Robot_Controller()
        # Clamping jaw tightening angle
        # 夹爪加紧角度
        self.grap_joint = self.robot.get_gripper_value(1)
        self._joint_5 = self.robot.joint5

    def move(self, joints, joints_down):
        '''
        移动过程 moving process
        :param joints: 移动到物体位置的各关节角度    The angle of each joint moving to the object position
        :param joints_down: 机械臂抬起各关节角度      The mechanical arm lifts each joint angle
        :param color_angle: 移动到对应垃圾桶的角度  Move to the angle corresponding to the trash can
        '''
        joints_uu = [90, 80, 50, 50, self._joint_5, self.grap_joint]
        # raise 抬起
        joints_up = [joints_down[0], 80, 50, 50, self._joint_5, 30]
        # Move over the object's position 移动至物体位置上方
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # Opening and closing jaws 开合夹爪
        for i in range(5):
            self.arm.Arm_serial_servo_write(6, 180, 100)
            sleep(0.1)
            self.arm.Arm_serial_servo_write(6, 30, 100)
            sleep(0.1)
        # Move to object position 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 1000)
        sleep(1.5)
        # Grasp and clamp the clamping claw进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(1)
        back_joints = [joints[0], joints[1]+30, joints[2]-30, joints[3]+10, joints[4], self.grap_joint]
        self.arm.Arm_serial_servo_write6_array(back_joints, 1000)
        sleep(1)
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1.5)
        # Lift to above the corresponding position 抬起至对应位置上方
        self.arm.Arm_serial_servo_write(1, joints_down[0], 1000)
        sleep(1.5)
        # Move to target location 移动至目标位置
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1.5)
        # Release the object and release the clamping jaws释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(1)
        # raise  抬起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1.5)

    def arm_run(self, name, joints):
        '''
        Manipulator movement function 机械臂移动函数
        :param name:识别的垃圾名称  Identified spam names
        :param joints: 反解求得的各关节角度 Angle of each joint obtained by inverse solution
        '''
        # Obtain the target joint angle 获得目标关节角
        joints = [joints[0], joints[1], joints[2], joints[3], joints[4], 30]
        # Hazardous waste - red
        # 有害垃圾--红色
        if name == "Syringe" or name == "Used_batteries" or name == "Expired_cosmetics" or name == "Expired_tablets" and self.move_status == True:
            # It is set here. You can only run down after this operation
            # 此处设置,需执行完本次操作,才能向下运行
            self.move_status = False
            # print("Hazardous waste")
            self.move(joints, self.robot.P_HAZARDOUS_WASTE)
            self.move_status = True
        # Recyclable waste - blue
        # 可回收垃圾--蓝色
        if name == "Zip_top_can" or name == "Newspaper" or name == "Old_school_bag" or name == "Book" and self.move_status == True:
            self.move_status = False
            # print("Recyclable waste")
            self.move(joints, self.robot.P_RECYCLABLE_WASTE)
            self.move_status = True
        # Kitchen waste - green
        # 厨余垃圾--绿色
        if name == "Fish_bone" or name == "Watermelon_rind" or name == "Apple_core" or name == "Egg_shell" and self.move_status == True:
            self.move_status = False
            # print("Kitchen waste")
            self.move(joints, self.robot.P_KITCHEN_WASTE)
            self.move_status = True

        # Other garbage--gray
        # 其他垃圾--灰色
        if name == "Cigarette_butts" or name == "Toilet_paper" or name == "Peach_pit" or name == "Disposable_chopsticks" and self.move_status == True:
            self.move_status = False
            # print("Other garbage")
            self.move(joints, self.robot.P_OTHER_WASTE)
            self.move_status = True
