from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    target_id = LaunchConfiguration('target_id', default='2.0')
    target_id_arg= DeclareLaunchArgument('target_id', default_value=target_id)

    apriltag_follow_2D_node = Node(
        package='largemodel_arm',
        executable='apriltag_follow_2D',
        name='apriltag_follow_2D',
        parameters=[{'target_id': target_id}]
    )
    gras_node = Node(
        package='M3Pro_demo',
        executable='grasp',
        name='grasp',
    )
    return LaunchDescription([
        target_id_arg,
        gras_node,#夹取节点
        apriltag_follow_2D_node,#大模型机器码分拣节点
    ])

































