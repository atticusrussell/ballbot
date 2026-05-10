import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node1= Node(
            package='dofbot_pro_info',       # ROS 2 包名称
            executable='kinemarics_dofbot',  # 节点执行文件
            name='dofbot_server',        # 节点名称
            output='screen',             # 输出到屏幕
        ) 
    return LaunchDescription([
       node1,
    ])
