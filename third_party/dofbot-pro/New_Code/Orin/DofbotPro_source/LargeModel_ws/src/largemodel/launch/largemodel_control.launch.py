import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
def generate_launch_description():
    # 获取包的共享目录
    cam_kin_demopkg_share = get_package_share_directory('dofbot_pro_info')
    params_file=os.path.join(get_package_share_directory('largemodel'), "config", "yahboom.yaml") 

    #启动参数
    text_chat_mode = LaunchConfiguration('text_chat_mode', default=False)
    text_chat_mode_arg= DeclareLaunchArgument('text_chat_mode', default_value=text_chat_mode)

    # 定义节点
    model_server = Node(
        package='largemodel',
        executable='model_service',
        name='model_service',
        parameters=[
            params_file,
            {'text_chat_mode': text_chat_mode}  # 动态参数，命令行参数覆盖yahboom.yaml同名参数再传给节点
        ],
        output='screen'
    )

    asr_server = Node(
        package='largemodel',
        executable='asr',
        name='asr',
        parameters=[params_file],
        output='screen',
        condition=UnlessCondition(text_chat_mode)
    )

    action_server = Node(
        package='largemodel',
        executable='action_service',
        name='action_service',
        parameters=[
            params_file,
            {'text_chat_mode': text_chat_mode}  # 动态参数，命令行参数覆盖yahboom.yaml同名参数再传给节点
        ],
        output='screen'
    )

    camrea_kin_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(cam_kin_demopkg_share, 'launch', 'camera_arm_kin.launch.py')))

    return LaunchDescription([
        text_chat_mode_arg,    #声明启动参数
        camrea_kin_node,       #启动相机和运动学结算节点
        model_server,          #启动模型服务节点
        action_server,         #启动动作服务器节
        asr_server,            #启动asr用户交互节点
    ])





