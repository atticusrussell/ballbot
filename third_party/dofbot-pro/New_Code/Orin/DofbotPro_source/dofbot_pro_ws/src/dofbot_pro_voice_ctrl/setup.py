from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'dofbot_pro_voice_ctrl'

extensions = ['*.csv', '*.xlsx','.text']
config_files = []
for ext in extensions:
    config_files.extend(glob(os.path.join('config', ext)))
    
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_recognition = dofbot_pro_voice_ctrl.voice_recognition:main',
            'voice_player = dofbot_pro_voice_ctrl.voice_player:main',
            'apriltag_detect = dofbot_pro_voice_ctrl.AprilTag.apriltag_detect:main',
            'apriltag_grasp_VC = dofbot_pro_voice_ctrl.AprilTag.apriltag_grasp_VC:main',
            'apriltag_list_VC = dofbot_pro_voice_ctrl.AprilTag.apriltag_list_VC:main',
            'apriltag_remove_higher_VC = dofbot_pro_voice_ctrl.AprilTag.apriltag_remove_higher_VC:main',
            'apriltag_follow_VC = dofbot_pro_voice_ctrl.AprilTag.apriltag_follow_VC:main',
            'color_detect_VC = dofbot_pro_voice_ctrl.Color.color_detect_VC:main',
            'color_grasp_VC = dofbot_pro_voice_ctrl.Color.color_grasp_VC:main',
            'color_hight_list = dofbot_pro_voice_ctrl.Color.color_hight_list:main',
            'remove_heigher_VC = dofbot_pro_voice_ctrl.Color.remove_heigher_VC:main',
            'color_follow_VC = dofbot_pro_voice_ctrl.Color.color_follow_VC:main',
            'msgToimg = dofbot_pro_voice_ctrl.Yolov11.msgToimg:main',
            'yolov11_sortation_VC = dofbot_pro_voice_ctrl.Yolov11.yolov11_sortation_VC:main',
            'compute_width = dofbot_pro_voice_ctrl.KCF.compute_width:main',
            'KCF_TrackAndGrap = dofbot_pro_voice_ctrl.KCF.KCF_TrackAndGrap:main',
            'KCF_Tracker = dofbot_pro_voice_ctrl.KCF.KCF_Tracker:main'
        ],
    },
)
