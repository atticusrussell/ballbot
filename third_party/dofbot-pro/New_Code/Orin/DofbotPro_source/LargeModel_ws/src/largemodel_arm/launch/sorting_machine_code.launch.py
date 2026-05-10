import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    target_id = LaunchConfiguration('target_id', default='2.0')
    target_id_arg= DeclareLaunchArgument('target_id', default_value=target_id)

    apriltag_sort_node = Node(
        package='largemodel_arm',
        executable='apriltag_sort',
        name='apriltag_sort',
        parameters=[{'target_id': target_id}]
    )
    grasp_desktop_node = Node(
        package='M3Pro_demo',
        executable='grasp_desktop',
        name='grasp_desktop',
    )
    return LaunchDescription([
        target_id_arg,
        grasp_desktop_node,#桌面夹取节点
        apriltag_sort_node,#大模型机器码分拣节点
    ])

































