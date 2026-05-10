#!/usr/bin/env python
# coding: utf-8
import rospy
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler

# Convert radians to degrees
# 弧度转换成角度
DE2RA = pi / 180

if __name__ == '__main__':
    rospy.init_node("dofbot_motion_plan_py")
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
    # 设置"down"为目标点
    dofbot.set_named_target("down")
    dofbot.go()
    sleep(0.5)
    # Create a pose instance
    # 创建位姿实例
    pos = Pose()
    # Set a specific location
    # 设置具体的位置
    pos.position.x = -0.15693421230179241
    pos.position.y = 0.17226876601519647
    pos.position.z = -0.01347811693107473
    # The unit of RPY is the angle value
    # RPY的单位是角度值
    roll = -140.0
    pitch = 0.0
    yaw = 0.0
    # RPY to Quaternion
    # RPY转四元数
    q = quaternion_from_euler(roll * DE2RA, pitch * DE2RA, yaw * DE2RA)
    pos.orientation.x = -0.9746898837164969
    pos.orientation.y = -0.0464761134797589
    pos.orientation.z = 0.13715776530975796
    pos.orientation.w = 0.1703154393229567
    #pos.orientation.x = q[0]
    #pos.orientation.y = q[1]
    #pos.orientation.z = q[2]
    #pos.orientation.w = q[3]
    # set target point
    # 设置目标点
    dofbot.set_pose_target(pos)
    # Execute multiple times to improve the success rate
    # 多次执行,提高成功率
    for i in range(5):
        # motion planning
        # 运动规划
        plan_success,plan,planning_time,error_code = dofbot.plan()
        if len(plan.joint_trajectory.points) != 0:
            print ("plan success")
            # Run after planning is successful
            # 规划成功后运行
            dofbot.execute(plan)
            break
        else:
            print ("plan error")
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
            
            
