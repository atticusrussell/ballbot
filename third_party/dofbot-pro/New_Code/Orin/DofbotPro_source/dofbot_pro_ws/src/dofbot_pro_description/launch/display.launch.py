from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  ld = LaunchDescription()

  default_urdf_path = FindPackageShare('dofbot_pro_description')
  default_model_path = PathJoinSubstitution(['urdf', 'DOFBOT_Pro-V24.urdf'])

  # These parameters are maintained for backwards compatibility
  gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                  description='Flag to enable joint_state_publisher_gui')
  ld.add_action(gui_arg)

  # This parameter has changed its meaning slightly from previous versions
  ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                      description='Path to robot urdf file relative to omnicar_description package'))

  ld.add_action(IncludeLaunchDescription(
    PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
    launch_arguments={
      'urdf_package': 'dofbot_pro_description',
      'urdf_package_path': LaunchConfiguration('model'),
      'jsp_gui': LaunchConfiguration('gui')}.items()
  ))

  return ld