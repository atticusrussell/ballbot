from setuptools import find_packages, setup

package_name = 'dofbot_pro_vision'

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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_qrcode = dofbot_pro_vision.create_qrcode:main',
            'parse_qrcode = dofbot_pro_vision.parse_qrcode:main',
            'detect_pose = dofbot_pro_vision.detect_pose:main',
            'detect_object = dofbot_pro_vision.detect_object:main',
            'simple_ar = dofbot_pro_vision.simple_ar:main',
            'astra_rgb_image = dofbot_pro_vision.astra_rgb_image:main',
            'astra_depth_image = dofbot_pro_vision.astra_depth_image:main',
            'pub_image = dofbot_pro_vision.pub_image:main',
        ],
    },
)
