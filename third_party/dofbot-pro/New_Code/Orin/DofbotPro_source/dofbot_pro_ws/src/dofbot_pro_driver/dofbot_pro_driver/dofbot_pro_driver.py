#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import degrees, pi
import numpy as np
from Arm_Lib import Arm_Device
import os
exit_code = os.system('ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 "data: 50"')
class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.Arm = Arm_Device()
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            100  # QoS: 队列长度
        )
        self.target_joints = ["Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint", "grip_joint"]
        self.joint_angles = [90, 90, 90, 90, 90, 180]  # 机械臂初始角度（单位：度）
        self.last_joint_angles = self.joint_angles[:]  # 记录上一次的角度
        self.get_logger().info("Subscribed to /joint_states")

    def joint_state_callback(self, msg):
        if not isinstance(msg, JointState):
            return

        joint_data = dict(zip(msg.name, msg.position))  # 关节名称 -> 角度（弧度）
        new_joint_angles = self.joint_angles[:]  # 复制当前角度

        DEG2RAD = 180 / pi  # 弧度转角度
        mid_offsets = np.array([90, 90, 90, 90, 90])  # 关节的基准角度

        # 处理前5个关节角度
        arm_rad = np.array([joint_data.get(j, 0) for j in self.target_joints[:5]])  # 获取关节值
        arm_deg = arm_rad * DEG2RAD  # 转换为角度
        new_joint_angles[:5] = (arm_deg + mid_offsets).tolist()  # 应用偏移

        # 处理夹爪角度
        if "grip_joint" in joint_data:
            grip_angle = degrees(joint_data["grip_joint"]) + 180  # 角度转换
            new_joint_angles[5] = np.interp(grip_angle, [90, 180], [180, 30])  # 插值映射

        # 只有当角度发生变化时才更新
        if new_joint_angles != self.last_joint_angles:
            self.get_logger().info(f"Updated Joint Angles: {new_joint_angles}")
            self.last_joint_angles = new_joint_angles[:]  # 更新上次的角度
            self.Arm.Arm_serial_servo_write6(
                new_joint_angles[0], new_joint_angles[1], new_joint_angles[2],
                new_joint_angles[3], new_joint_angles[4], new_joint_angles[5], time=1000)

        self.joint_angles = new_joint_angles  # 更新当前角度

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
