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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Retrieve the absolute path to the Fast DDS XML (fastrtps.xml) from the ballbot_base package
    ballbot_base_share = get_package_share_directory('ballbot_base')
    fastrtps_xml_file = os.path.join(ballbot_base_share, 'config', 'fastrtps.xml')

    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('ballbot_bringup'), 'launch', 'sensors.launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('ballbot_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('ballbot_description'), 'launch', 'description.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("ballbot_base"), "config", "ekf.yaml"]
    )

    twist_mux_config_path = PathJoinSubstitution(
        [FindPackageShare("ballbot_base"), "config", "twist_mux.yaml"]
    )

    robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('ballbot_bringup'), 'launch', 'robot.launch.py']
    )

    return LaunchDescription([
        # Set the environment variables so that Fast DDS uses the custom XML profile.
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastrtps_xml_file),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),

        DeclareLaunchArgument(
            name='base_serial_port',
            default_value='/dev/ttyACM0',
            description='Linorobot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='joy', 
            default_value='true',
            description='Use Joystick'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux_node',
            output='screen',
            parameters=[twist_mux_config_path],
            remappings=[('/cmd_vel_out', '/cmd_vel_muxed')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch_path),
            launch_arguments={
                'base_serial_port': LaunchConfiguration("base_serial_port")
            }.items()
        )
    ])
