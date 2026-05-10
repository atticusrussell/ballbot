#!/usr/bin/env python
# coding: utf-8
import rospy, sys
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

import Arm_Lib
from sensor_msgs.msg import JointState

# Convert radians to degrees
# 弧度转换成角度
DE2RA = pi / 180
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
	
	
	
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Set_Scene')
    # Initialize the scene object
    # 初始化场景对象
    sbus = Arm_Lib.Arm_Device()
    scene = PlanningSceneInterface()
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
    # Set "up" as the target point
    # 设置"up"为目标点
    dofbot.set_named_target("up")
    dofbot.go()
    sleep(0.5)
    target_joints1 = [0, -1.18, -1.17, 0.77, 0.03]
    target_joints2 = [0, -1.21, 0.52, -0.89, 0.08]
    tool_size = [0.03, 0.03, 0.03]
    # Get the name of the terminal link
    # 获取终端link的名称
    end_effector_link = dofbot.get_end_effector_link()
    # Set the pose of the tool
    # 设置tool的位姿
    p = PoseStamped()
    p.header.frame_id = end_effector_link
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.0
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1
    # Attach the tool to the gripper of the robotic arm
    # 将tool附着到机械臂的夹爪上
    scene.attach_box(end_effector_link, 'tool', p, tool_size)
    
    subscriber = rospy.Subscriber("/joint_states", JointState, topic)
    
    while 1:
        dofbot.set_joint_value_target(target_joints1)
        dofbot.go()
        sleep(0.5)
        
        dofbot.set_joint_value_target(target_joints2)
        dofbot.go()
        sleep(0.5)
        
        
        
        
        
        
        
        
