#!/usr/bin/env python
# coding: utf-8
import rospy
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler

# 角度转弧度
DE2RA = pi / 180

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node("set_pose_py")
    # 初始化机械臂
    dofbot_pro = MoveGroupCommander("arm_group")
    # 当运动规划失败后，允许重新规划
    dofbot_pro.allow_replanning(True)
    dofbot_pro.set_planning_time(5)
    # 尝试规划的次数
    dofbot_pro.set_num_planning_attempts(10)
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    dofbot_pro.set_goal_position_tolerance(0.01)
    dofbot_pro.set_goal_orientation_tolerance(0.01)
    # 设置允许目标误差
    dofbot_pro.set_goal_tolerance(0.01)
    # 设置允许的最大速度和加速度
    dofbot_pro.set_max_velocity_scaling_factor(1.0)
    dofbot_pro.set_max_acceleration_scaling_factor(1.0)
    # 设置"down"为目标点
    dofbot_pro.set_named_target("down")
    dofbot_pro.go()
    sleep(0.5)
    # 创建位姿实例
    pos = Pose()
    # 设置具体的位置
    pos.position.x = -0.000599999999999989
    pos.position.y = 0.1835691951077983
    pos.position.z = 0.2550887465333698

    pos.orientation.x = 0.017276103846364667
    pos.orientation.y = 7.852827837256886e-05
    pos.orientation.z =  1.3568651988846686e-06
    pos.orientation.w = 0.9998507538964794

    # 设置目标点
    dofbot_pro.set_pose_target(pos)
    print("-------------------------")
    #plan = dofbot_pro.plan()
    # 多次执行,提高成功率
    for i in range(5):
        print("-------------------------")
        # 运动规划
        plan = dofbot_pro.plan()
        print("-5555555555555555555555")
        #print(plan.joint_trajectory.points)
        if len(plan.joint_trajectory.points) != 0:
            print ("plan success")
            # 规划成功后运行
            dofbot_pro.execute(plan)
            break
        else: print ("plan error")
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

