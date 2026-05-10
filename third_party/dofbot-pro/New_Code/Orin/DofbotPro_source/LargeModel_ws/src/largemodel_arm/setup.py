from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'largemodel_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'apriltag_follow_2D = largemodel_arm.apriltag_follow_2D:main',    
            'apriltag_remove_higher = largemodel_arm.apriltag_remove_higher:main', 
            'apriltag_sort = largemodel_arm.apriltag_sort:main', 
            'apriltag_transport_ALM = largemodel_arm.apriltag_transport_ALM:main', 
            'color_follow_2D = largemodel_arm.color_follow_2D:main', 
            'color_remove_higher = largemodel_arm.color_remove_higher:main', 
            'color_sort = largemodel_arm.color_sort:main', 
            'color_transport_ALM = largemodel_arm.color_transport_ALM:main', 
            'send_goal_ALM = largemodel_arm.send_goal_ALM:main', 
            'grasp_desktop_remove = largemodel_arm.grasp_desktop_remove:main', 
            'grasp_desktop = largemodel_arm.grasp_desktop:main', 
            'grasp = largemodel_arm.grasp:main', 
            'KCF_follow = largemodel_arm.KCF_follow:main', 
            'follow_line = largemodel_arm.follow_line:main', 
            'grasp_desktop_remove_color = largemodel_arm.grasp_desktop_remove_color:main', 
            'KCF_track = largemodel_arm.KCF_track:main', 
            'grasp_desktop_apritag = largemodel_arm.grasp_desktop_apritag:main',
            'KCF_Track_Move = largemodel_arm.KCF_Track_Move:main',
            'KCF_Grap_Move = largemodel_arm.KCF_Grap_Move:main',
            'ALM_KCF_Tracker = largemodel_arm.ALM_KCF_Tracker:main',
            'yolov11_sortation = largemodel_arm.yolov11_sortation:main',
            'yolov11_ALM = largemodel_arm.yolov11_ALM:main',
            'Get_Target_Pose_KCF = largemodel_arm.Get_Target_Pose_KCF:main',
            'Record_Video = largemodel_arm.Record_Video:main',
            'Record_pose = largemodel_arm.Record_pose:main',
            'Stack_Grap_Move = largemodel_arm.Stack_Grap_Move:main',
            'ALM_Point_To = largemodel_arm.ALM_Point_To:main'
        ],
    },
)
