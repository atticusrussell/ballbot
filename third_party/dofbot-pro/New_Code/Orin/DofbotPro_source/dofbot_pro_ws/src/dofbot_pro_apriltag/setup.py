from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'dofbot_pro_apriltag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'param'),glob(os.path.join('param', '*.yaml'))),
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
            'msgToimg = dofbot_pro_apriltag.msgToimg:main',
            'apriltagID_finger_detect = dofbot_pro_apriltag.apriltagID_finger_detect:main',
            'MediapipeGesture = dofbot_pro_apriltag.MediapipeGesture:main',
            'apriltag_list_Hight = dofbot_pro_apriltag.apriltag_list_Hight:main',
            'apriltag_list_Dist = dofbot_pro_apriltag.apriltag_list_Dist:main',
            'apriltag_follow = dofbot_pro_apriltag.apriltag_follow:main',
            'calibrate_offset = dofbot_pro_apriltag.calibrate_offset:main'
        ],
    },
)
