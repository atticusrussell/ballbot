from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
def generate_launch_description():
    grasp_desktop_node = Node(
        package='M3Pro_demo',
        executable='grasp_desktop',
        name='grasp_desktop',
    )


    return LaunchDescription([
        grasp_desktop_node,#桌面夹取节点
    ])























































