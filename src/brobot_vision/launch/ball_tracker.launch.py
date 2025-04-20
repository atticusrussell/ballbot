import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    sim_mode_dec = DeclareLaunchArgument(
        name='sim', 
        default_value='false',
        description='Use simulation tracking parameters'
    )

    # Access the value of the launch argument
    sim = LaunchConfiguration('sim')


    tracker_params_sim = PathJoinSubstitution([
        FindPackageShare("brobot_vision"),
        "config",
        "ball_tracker_params_sim.yaml"
    ])

    tracker_params_robot = PathJoinSubstitution([
        FindPackageShare("brobot_vision"),
        "config",
        "ball_tracker_params_robot.yaml"
    ])

    ball_tracker_launch_path = PathJoinSubstitution([
        FindPackageShare('ball_tracker'),
        'launch', 
        'ball_tracker.launch.py'
    ])

    # --- Select the parameter file based on simulation flag ---
    params_path = PythonExpression([
        '"', tracker_params_sim,
        '" if "true" == "', sim,
        '" else "', tracker_params_robot,
        '"'
    ])


    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ball_tracker_launch_path),
        launch_arguments={
            'params_file': params_path,
            'image_topic': '/camera/image_raw',
            'cmd_vel_topic': '/cmd_vel_tracker',
            'enable_3d_tracker': 'true'
        }.items()
    )

    return LaunchDescription([
        sim_mode_dec,
        tracker_launch,
    ])
