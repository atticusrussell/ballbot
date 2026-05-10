#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dofbot_pro_interface.msg import ImageMsg 

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')  # ROS2节点必须继承自Node
        
        # 配置QoS策略（匹配ROS1的默认行为）
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.bridge = CvBridge()
        
        # 参数声明和获取
        self.declare_parameter('img_flip', False)
        self.img_flip = self.get_parameter('img_flip').value
        
        # 创建订阅者（ROS2需要显式指定消息类型）
        self.image_sub = self.create_subscription(Image,'/camera/color/image_raw',self.image_sub_callback,qos_profile)
        # 创建发布者
        self.image_pub = self.create_publisher(ImageMsg,'/image_data', 1)
        
        # 初始化消息对象
        self.image_msg = ImageMsg()
        self.last_img = np.zeros((480, 640, 3), dtype=np.uint8)

    def image_sub_callback(self, msg):
        try:
            # ROS2消息处理需要放在try-catch块中
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # 图像翻转处理
            if self.img_flip:
                cv_image = cv2.flip(cv_image, 1)
                
            # 更新消息内容
            self.image_msg.height = cv_image.shape[0]
            self.image_msg.width = cv_image.shape[1]
            self.image_msg.channels = cv_image.shape[2]
            self.image_msg.data = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8').data
            
            self.image_pub.publish(self.image_msg)
            self.last_img = cv_image
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    try:
        image_listenning = ImageListener()
        rclpy.spin(image_listenning)
    except KeyboardInterrupt:
        image_listenning.get_logger().info("Shutting down...")
    finally:
        image_listenning.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()