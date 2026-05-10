#!/usr/bin/env python3
# coding: utf-8
'''
执行此段代码-->订阅发布话题为"joint_states"的各关节角度,驱动真机移动
Execute this code-->Subscribe and publish the joint angles of the topic "joint states" to drive the real machine to move
'''
import rospy
import Arm_Lib
from math import pi
from sensor_msgs.msg import JointState

# Convert radians to degrees
# 弧度转换成角度
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
	sbus = Arm_Lib.Arm_Device()
	rospy.init_node("ros_dofbot")
	subscriber = rospy.Subscriber("/joint_states", JointState, topic)
	rate = rospy.Rate(2)
	rospy.spin()
