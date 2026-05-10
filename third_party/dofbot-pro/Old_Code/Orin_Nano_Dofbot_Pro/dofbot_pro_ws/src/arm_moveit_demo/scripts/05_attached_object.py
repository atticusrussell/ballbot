#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from time import sleep
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('attached_object_py')
    scene = PlanningSceneInterface()
    dofbot_pro = MoveGroupCommander('arm_group')
    # 当运动规划失败后，允许重新规划
    dofbot_pro.allow_replanning(True)
    dofbot_pro.set_planning_time(5)
    dofbot_pro.set_num_planning_attempts(10)
    dofbot_pro.set_goal_position_tolerance(0.01)
    dofbot_pro.set_goal_orientation_tolerance(0.01)
    dofbot_pro.set_goal_tolerance(0.01)
    dofbot_pro.set_max_velocity_scaling_factor(1.0)
    dofbot_pro.set_max_acceleration_scaling_factor(1.0)
    dofbot_pro.set_named_target("up")
    dofbot_pro.go()
    sleep(0.5)
    # 设置障碍物的三维尺寸[长宽高]
    table_size = [0.1, 0.2, 0.02]
    table_pose = PoseStamped()
    table_pose.header.frame_id = 'base_link'
    table_pose.pose.position.x = 0.0
    table_pose.pose.position.y = 0.2
    table_pose.pose.position.z = 0.25 + table_size[2] / 2.0
    table_pose.pose.orientation.w = 1.0
    scene.add_box('obj', table_pose, table_size)
    rospy.sleep(2)
    dofbot_pro.set_named_target("down")
    dofbot_pro.go()
    sleep(0.5)
    index = 0
    while index < 10:
        # 设置随机目标点
        dofbot_pro.set_random_target()
        # 开始运动
        dofbot_pro.go()
        sleep(0.5)
        index += 1
        print ("第 {} 次规划!!!".format(index))
    scene.remove_attached_object('obj')
    scene.remove_world_object()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
