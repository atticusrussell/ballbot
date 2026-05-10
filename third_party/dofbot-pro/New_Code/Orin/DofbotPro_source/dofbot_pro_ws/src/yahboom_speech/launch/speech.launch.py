from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    play_node = Node(
     package='yahboom_speech',
     executable='voice_player',
     name='voice_player_node',
    )

    recognize_node = Node(
     package='yahboom_speech',
     executable='speech_recognize',
     name='speech_recognize_node',
    )
    
    return LaunchDescription([play_node,recognize_node])