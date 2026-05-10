#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from Arm_Lib import Arm_Device
from dofbot_pro_interface.msg import ArmJoint
from std_msgs.msg import Bool
import time
from sensor_msgs.msg import JointState
import numpy as np
from math import pi
import os
exit_code = os.system('ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 "data: 50"')
class ArmDriver(Node):
    def __init__(self):
        super().__init__('arm_driver_node')
        self.Arm = Arm_Device()
        
        # 参数声明
        self.declare_parameter('prefix', '')
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value
        
        # 订阅器和发布器
        self.sub_arm = self.create_subscription(
            ArmJoint, 
            'TargetAngle', 
            self.arm_callback, 
            qos_profile=1000
        )
        
        self.sub_buzzer = self.create_subscription(
            Bool,
            'Buzzer',
            self.buzzer_callback,
            qos_profile=1000
        )
        
        self.arm_pub_update = self.create_publisher(
            ArmJoint, 
            'ArmAngleUpdate', 
            qos_profile=1000
        )
        
        # 初始化参数
        self.joints = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
        self.cur_joints = [90.0, 90.0, 90.0, 0.0, 90.0, 30.0]
        self.RA2DE = 180 / pi
        
        # 初始化机械臂位置
        self.Arm.Arm_serial_servo_write6(90.0, 90.0, 90.0, 0.0, 90.0, 30.0, 3000)

    def arm_callback(self, msg):
        arm_joint = ArmJoint()
        
        if len(msg.joints) != 0:
            self.get_logger().info(f"Received joints: {msg.joints}")
            arm_joint.joints = self.cur_joints
            for _ in range(2):
                self.Arm.Arm_serial_servo_write6(
                    msg.joints[0], msg.joints[1], msg.joints[2],
                    msg.joints[3], msg.joints[4], msg.joints[5],
                    time=msg.run_time
                )
                self.cur_joints = list(msg.joints)
                self.arm_pub_update.publish(arm_joint)
        else:
            self.get_logger().info(f"Moving joint {msg.id} to {msg.angle}")
            arm_joint.id = msg.id
            arm_joint.angle = msg.angle
            for _ in range(2):
                self.Arm.Arm_serial_servo_write(msg.id, msg.angle, msg.run_time)
                self.cur_joints[msg.id - 1] = msg.angle
                self.arm_pub_update.publish(arm_joint)
        
        self.joints_states_update()

    def buzzer_callback(self, msg):
        if msg.data:
            self.get_logger().info("Buzzer ON")
            self.Arm.Arm_Buzzer_On()
        else:
            self.get_logger().info("Buzzer OFF")
            self.Arm.Arm_Buzzer_Off()

    def read_current_joint(self):
        for i in range(6):
            time.sleep(0.01)
            self.cur_joints[i] = self.Arm.Arm_serial_servo_read(i + 1)
            time.sleep(0.01)
        self.joints_states_update()

    def joints_states_update(self):
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "joint_states"
        
        if self.prefix:
            state.name = [f"{self.prefix}/Arm1_Joint", f"{self.prefix}/Arm2_Joint",
                          f"{self.prefix}/Arm3_Joint", f"{self.prefix}/Arm4_Joint",
                          f"{self.prefix}/Arm5_Joint", f"{self.prefix}/grip_joint"]
        else:
            state.name = ["Arm1_Joint", "Arm2_Joint", "Arm3_Joint", 
                          "Arm4_Joint", "Arm5_Joint", "grip_joint"]
        
        joints = self.cur_joints.copy()
        joints[5] = np.interp(joints[5], [30, 180], [0, 90])
        mid = np.array([90, 90, 90, 90, 90, 90])
        array = np.array(joints) - mid
        position_src = (array * pi / 180).tolist()
        state.position = position_src
        
        #self.sta_publisher.publish(state)

def main(args=None):
    rclpy.init(args=args)
    node = ArmDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()