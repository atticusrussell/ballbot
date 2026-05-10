from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    camera_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('orbbec_camera'), 'launch'),
         '/dabai_dcw2.launch.py'])
    )

    kin_node = Node(
     package='dofbot_pro_info',
     executable='kinemarics_dofbot',
     name='kinemarics_dofbot',
    )

    
    return LaunchDescription([kin_node,camera_driver_launch])

