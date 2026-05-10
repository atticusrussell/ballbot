from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    target_high = LaunchConfiguration('target_high', default='4.0')
    target_high_arg= DeclareLaunchArgument('target_high', default_value=target_high)
    largemodel_armpkg_share = get_package_share_directory('largemodel_arm')
    
    apriltag_remove_higher_node = Node(
        package='largemodel_arm',
        executable='apriltag_remove_higher',
        name='apriltag_remove_higher',
        parameters=[{'target_high': target_high}]
    )
    grasp_desktop_node = Node(
        package='M3Pro_demo',
        executable='grasp_desktop',
        name='grasp_desktop',
    )

    delayed_apriltag_remove_higher_node = TimerAction(
        period=10.0,  # 等待 3 秒后启动
        actions=[
            Node(
                package='largemodel_arm',
                executable='apriltag_remove_higher',
                name='apriltag_remove_higher',
                parameters=[{'target_high': target_high}]
            )
        ]
    )
    camrea_kin_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(largemodel_armpkg_share, 'launch', 'bringup_grasp_desktop.launch.py')))
    return LaunchDescription([
        target_high_arg,
        grasp_desktop_node,#桌面夹取节点
        delayed_apriltag_remove_higher_node,#大模型机器码分拣节点
    ])























































