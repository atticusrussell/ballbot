#!/usr/bin/env python
# coding: utf-8
from time import sleep
import rospy
from moveit_commander.move_group import MoveGroupCommander

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
