#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
from time import sleep
import moveit_commander
from copy import deepcopy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_plan_py')
    scene = PlanningSceneInterface()
    yahboomcar = MoveGroupCommander('dofbot')
    # 当运动规划失败后，允许重新规划
    yahboomcar.allow_replanning(True)
    yahboomcar.set_planning_time(50)
    yahboomcar.set_num_planning_attempts(20)
    yahboomcar.set_goal_position_tolerance(0.01)
    yahboomcar.set_goal_orientation_tolerance(0.01)
    yahboomcar.set_goal_tolerance(0.01)
    yahboomcar.set_max_velocity_scaling_factor(1.0)
    yahboomcar.set_max_acceleration_scaling_factor(1.0)
    rospy.loginfo("Set Init Pose")
    joints = [0, -1.57, -0.74, 0.71, 0]
    yahboomcar.set_joint_value_target(joints)
    yahboomcar.execute(yahboomcar.plan())
    sleep(0.5)
    # 获取当前位姿数据最为机械臂运动的起始位姿
    end_effector_link = yahboomcar.get_end_effector_link()
    start_pose = yahboomcar.get_current_pose(end_effector_link).pose
    # 初始化路点列表
    waypoints = []
    # 如果为True,将初始位姿加入路点列表
    waypoints.append(start_pose)
    for i in range(3):
        # 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        wpose.position.z += 0.13
        waypoints.append(deepcopy(wpose))
        wpose.position.z -= 0.13
        waypoints.append(deepcopy(wpose))
    # 规划过程
    fraction = 0.0  # 路径规划覆盖率
    maxtries = 100  # 最大尝试规划次数
    attempts = 0    # 已经尝试规划次数
    rospy.loginfo("Path Planning in Cartesian Space")
    # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
    while fraction < 1.0 and attempts < maxtries:
        '''
        waypoints: 路点列表
        eef_step: 终端步进值，每隔0.1m计算一次逆解判断能否可达
        jump_threshold: 跳跃阈值，设置为0代表不允许跳跃
        plan: 路径, fraction: 路径规划覆盖率
        '''
        (plan, fraction) = yahboomcar.compute_cartesian_path(waypoints, 0.1, 0.0, True)
        attempts += 1
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the yahboomcar.")
        yahboomcar.execute(plan)
        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo("Path planning failed with only " + str(
            fraction) + " success after " + str(maxtries) + " attempts.")
    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

