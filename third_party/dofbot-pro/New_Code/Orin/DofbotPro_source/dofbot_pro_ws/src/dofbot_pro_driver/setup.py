from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dofbot_pro_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),glob(os.path.join('config', '*.yaml'))),
     
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
        	'dofbot_pro_driver = dofbot_pro_driver.dofbot_pro_driver:main',
        	'arm_driver = dofbot_pro_driver.arm_driver:main',
            'calculate_volume = dofbot_pro_driver.calculate_volume:main',
            'grasp = dofbot_pro_driver.grasp:main',
            'apriltag_detect = dofbot_pro_driver.apriltag_detect:main',
            'test = dofbot_pro_driver.test:main',
            'apriltag_list = dofbot_pro_driver.apriltag_list:main',
            'apriltag_remove_higher = dofbot_pro_driver.apriltag_remove_higher:main'
           
        ],
    },
)
