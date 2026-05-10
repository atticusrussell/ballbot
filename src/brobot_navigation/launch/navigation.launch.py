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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory


MAP_NAME='upstairs' #change to the name of your own map here

def generate_launch_description():
    # Retrieve the absolute path to the Fast DDS XML (fastrtps.xml) from the brobot_base package
    brobot_base_share = get_package_share_directory('brobot_base')
    fastrtps_xml_file = os.path.join(brobot_base_share, 'config', 'fastrtps.xml')

    depth_sensor = os.getenv('BROBOT_DEPTH_SENSOR', '')

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('brobot_navigation'), 'rviz', 'brobot_navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('brobot_navigation'), 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('brobot_navigation'), 'config', 'navigation.yaml']
    )

    return LaunchDescription([
        # Set the environment variables so that Fast DDS uses the custom XML profile.
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastrtps_xml_file),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),

        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

       DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        GroupAction([
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                launch_arguments={
                    'map': LaunchConfiguration("map"),
                    'use_sim_time': LaunchConfiguration("sim"),
                    'params_file': nav2_config_path
                }.items()
            ),
        ]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])