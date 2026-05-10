#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import pi
import rospy, rospkg
from time import sleep
import moveit_commander
from moveit_msgs.msg import PlanningSceneWorld
from yahboomcar_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, PlanningScene, PlannerInterfaceDescription
from sensor_msgs.msg import JointState


def add_obj(table_pose, obj, table_size, xyz):
    table_pose.header.frame_id = 'base_footprint'
    table_pose.pose.position.x = xyz[0]
    table_pose.pose.position.y = xyz[1]
    table_pose.pose.position.z = xyz[2]
    table_pose.pose.orientation.w = 1.0
    scene.add_box(obj, table_pose, table_size)



if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Set_Scene')
    # 仿真
    pub_joint = rospy.Publisher("/move_group/fake_controller_joint_states", JointState, queue_size=1000)
    # 真机
    pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
    arm_joint = ArmJoint()
    arm_joint.id = 6
    arm_joint.angle = 180 - 0.55 * 180 / pi
    joint_state = JointState()
    joint_state.name = ["grip_joint"]
    joint_state.position = [-0.58]
    for i in range(10):
        pub_joint.publish(joint_state)
        pub_Arm.publish(arm_joint)
        sleep(0.1)
    yahboomcar = MoveGroupCommander('dofbot')
    end_effector_link = yahboomcar.get_end_effector_link()
    scene = PlanningSceneInterface()
    scene.remove_attached_object(end_effector_link, "tool")
    scene.remove_world_object()
    yahboomcar.allow_replanning(True)
    yahboomcar.set_planning_time(1)
    yahboomcar.set_num_planning_attempts(10)
    yahboomcar.set_goal_position_tolerance(0.01)
    yahboomcar.set_goal_orientation_tolerance(0.01)
    yahboomcar.set_goal_tolerance(0.01)
    yahboomcar.set_max_velocity_scaling_factor(1.0)
    yahboomcar.set_max_acceleration_scaling_factor(1.0)
    rospy.loginfo("Set Init Pose")
    yahboomcar.set_named_target("init")
    yahboomcar.go()
    p = PoseStamped()
    p.header.frame_id = end_effector_link
    p.pose.orientation.w = 1
    # 添加tool
    scene.attach_box(end_effector_link, 'tool', p, [0.03, 0.03, 0.03])
    target_joints1 = [0, -1.18, -1.17, 0.77, 0]
    target_joints2 = [0, -1.21, 0.52, -0.89, 0]
    table_list = {
        "obj0": [[0.08, 0.01, 0.4], [0.4, -0.1, 0.2]],
        "obj1": [[0.08, 0.01, 0.4], [0.4, 0.1, 0.2]],
        "obj2": [[0.08, 0.22, 0.01], [0.4, 0, 0.4]],
        "obj3": [[0.08, 0.22, 0.01], [0.4, 0, 0.29]],
        "obj4": [[0.08, 0.22, 0.01], [0.4, 0, 0.17]],
    }
    # 添加obj
    for i in range(len(table_list)):
        add_obj(p, table_list.keys()[i], table_list[table_list.keys()[i]][0],
                table_list[table_list.keys()[i]][1])
    rospy.loginfo("Grip Target")
    i = 0
    while i < 5:
        yahboomcar.set_joint_value_target(target_joints1)
        yahboomcar.go()
        yahboomcar.set_joint_value_target(target_joints2)
        yahboomcar.go()
        i += 1
        print ("第 {} 次规划!!!".format(i))
    scene.remove_attached_object(end_effector_link, 'tool')
    scene.remove_world_object()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
