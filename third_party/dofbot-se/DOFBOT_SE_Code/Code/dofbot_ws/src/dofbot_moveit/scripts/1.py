#!/usr/bin/env python
# coding: utf-8
from time import sleep
import rospy
from moveit_commander.move_group import MoveGroupCommander
from math import pi
import Arm_Lib
from sensor_msgs.msg import JointState
RA2DE = 180 / pi

def topic(msg):
    # If it is not the data of the topic, return it directly
    # 如果不是该话题的数据直接返回
	if not isinstance(msg, JointState): return
    # Define the joint angle container, the last one is the angle of the gripper, the default gripper does not move to 90.
    # 定义关节角度容器,最后一个是夹爪的角度,默认夹爪不动为90.
	joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Convert received radians [-1.57, 1.57] to degrees [0, 180]
    # 将接收到的弧度[-1.57,1.57]转换成角度[0,180]
	for i in range(6): 
		joints[i] = (msg.position[i] * RA2DE) + 90
		if(i == 5):
			joints[i] = (msg.position[i] * 116) + 180
    # Tuning the driver function
    # 调驱动函数
	sbus.Arm_serial_servo_write6_array(joints, 100)
if __name__ == '__main__':
    rospy.init_node("dofbot_random_move")
    # Initialize the robotic arm motion planning group
    # 初始化机械臂运动规划组
    dofbot = MoveGroupCommander("dofbot")
    # Allow replanning when motion planning fails
    # 当运动规划失败后，允许重新规划
    dofbot.allow_replanning(True)
    dofbot.set_planning_time(5)
    # number of attempts to plan
    # 尝试规划的次数
    dofbot.set_num_planning_attempts(10)
    # Set allowable target position error
    # 设置允许目标位置误差
    dofbot.set_goal_position_tolerance(0.01)
    # Set the allowable target attitude error
    # 设置允许目标姿态误差
    dofbot.set_goal_orientation_tolerance(0.01)
    # Set allowable target error
    # 设置允许目标误差
    dofbot.set_goal_tolerance(0.01)
    # set maximum speed
    # 设置最大速度
    dofbot.set_max_velocity_scaling_factor(1.0)
    # set maximum acceleration
    # 设置最大加速度
    dofbot.set_max_acceleration_scaling_factor(1.0)
    sbus = Arm_Lib.Arm_Device()
    subscriber = rospy.Subscriber("/joint_states", JointState, topic)
    while (1):
        # Set random target points
        # 设置随机目标点
        dofbot.set_random_target()
        # start exercising
        # 开始运动   
        dofbot.go()
        sleep(0.5)
        # Set "up" as the target point
        # 设置"up"为目标点
        #dofbot.set_named_target("up")
        #dofbot.go()
        #sleep(0.5)
        # Set "down" as the target point
        # 设置"down"为目标点
        #dofbot.set_named_target("down")
        #dofbot.go()
        #sleep(0.5)
