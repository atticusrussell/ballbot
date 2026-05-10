#!/usr/bin/env python
# coding: utf-8
import rospy
from math import pi
from time import sleep
from moveit_commander.move_group import MoveGroupCommander

# 角度转弧度
DE2RA = pi / 180

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node("set_joint_py", anonymous=True)
    # 初始化机械臂
    dofbot_pro = MoveGroupCommander("arm_group")
    # 当运动规划失败后，允许重新规划
    dofbot_pro.allow_replanning(True)
    dofbot_pro.set_planning_time(5)
    # 尝试规划的次数
    dofbot_pro.set_num_planning_attempts(10)
    # 设置允许目标角度误差
    dofbot_pro.set_goal_joint_tolerance(0.001)
    # 设置允许的最大速度和加速度
    dofbot_pro.set_max_velocity_scaling_factor(1.0)
    dofbot_pro.set_max_acceleration_scaling_factor(1.0)
    # 设置"down"为目标点
    dofbot_pro.set_named_target("down")
    dofbot_pro.go()
    sleep(0.5)
    # 设置目标点 弧度
    joints = [0.79, 0.79, -1.57, -1.57, 0]
    dofbot_pro.set_joint_value_target(joints)
    # 多次执行,提高成功率
    for i in range(5):
        # 运动规划
        plan = dofbot_pro.plan()
        #print("plan: ",plan)
        if len(plan.joint_trajectory.points) != 0:
            print ("plan success")
            # 规划成功后运行
            dofbot_pro.execute(plan)
            break
        else:
            print ("plan error")
