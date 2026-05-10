#!/usr/bin/env python3
# coding: utf-8
from time import sleep

import rospy
import numpy as np
from math import pi
from dofbot_pro_info.msg import *
from sensor_msgs.msg import JointState
import sys
np.set_printoptions(threshold=sys.maxsize)

class SimulationToMachine:
    def __init__(self):
        self.joints = [90.0, 90.0, 90.0, 90.0, 90.0, 30.0]
        self.subscriber = rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, self.topic)
        self.pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
        sleep(0.1)
        self.pubArm(self.joints)

    def topic(self, msg):
        # 如果不是该话题的数据直接返回
        if not isinstance(msg, JointState): return
        arm_rad = np.array(msg.position)
        DEG2RAD = np.array([180 / pi])
        arm_deg = np.dot(arm_rad.reshape(-1, 1), DEG2RAD)
        if len(msg.position) == 5:
            mid = np.array([90, 90, 90, 90, 90])
            #mid = np.array([0, 0, 0, 0, 0])
            arm_array = np.array(np.array(arm_deg) + mid)
            for i in range(5): self.joints[i] = arm_array[i]
        elif len(msg.position) == 1:
            # arm_deg: -88~0 ;arm_array: 91~180
            arm_array = np.array(np.array(arm_deg) + np.array([180]))
            self.joints[5] = np.interp(arm_array, [90, 180], [30, 180])[0]
        self.pubArm(self.joints)

    def pubArm(self, joints, run_time=1000):
        arm_joint = ArmJoint()
        arm_joint.joints = joints
        arm_joint.run_time = run_time
        self.pub_Arm.publish(arm_joint)


if __name__ == '__main__':
    rospy.init_node("moveit_to_machine")
    SimulationToMachine()
    rate = rospy.Rate(2)
    rospy.spin()
