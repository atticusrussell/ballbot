# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    laser_sensor_name = os.getenv('BROBOT_LASER_SENSOR', '')
    base_laser_sensor_name = os.getenv('BROBOT_BASE_LASER_SENSOR', '')
    
    fake_laser_config_path = PathJoinSubstitution(
        [FindPackageShare('brobot_bringup'), 'config', 'fake_laser.yaml']
    )

    

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare('brobot_bringup'), 'launch', 'lasers.launch.py']
    )

    

    camera_launch_path = PathJoinSubstitution(
        [FindPackageShare('brobot_bringup'), 'launch', 'camera.launch.py']
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path),
            condition=IfCondition(PythonExpression(['"" != "', laser_sensor_name, '"'])),
            launch_arguments={'sensor': laser_sensor_name}.items()   
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path),
            condition=IfCondition(PythonExpression(['"" != "', base_laser_sensor_name, '"'])),
            launch_arguments={
                'sensor': base_laser_sensor_name,
                'topic_name': 'base/scan',
                'frame_id': 'base_laser'
            }.items()   
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path)
        ),   
    ])

