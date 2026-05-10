from setuptools import find_packages, setup

package_name = 'dofbot_pro_yolov11'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'yolov11_sortation = dofbot_pro_yolov11.yolov11_sortation:main',
            'msgToimg = dofbot_pro_yolov11.msgToimg:main'
        ],
    },
)
