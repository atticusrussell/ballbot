#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import pi

import rospy, sys
from time import sleep
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Set_Scene')
    # Initialize the scene object
    # 初始化场景对象
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
    target_joints1 = [0, -1.18, -1.17, 0.79, 0.03]
    target_joints3 = [0, 0.70, -1.57, -1.57, 0.03]
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
    i = 0
    while i < 5:
        dofbot.set_joint_value_target(target_joints1)
        dofbot.go()
        sleep(0.5)
        dofbot.set_joint_value_target(target_joints2)
        dofbot.go()
        sleep(0.5)
        i += 1
        print ("第 {} 次规划!!!".format(i))
    scene.remove_attached_object(end_effector_link, 'tool')
    scene.remove_world_object()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
        
        
        
        
        
