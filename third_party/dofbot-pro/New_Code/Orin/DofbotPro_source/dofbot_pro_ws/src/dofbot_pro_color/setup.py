from setuptools import find_packages, setup

package_name = 'dofbot_pro_color'

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
            'color_detect = dofbot_pro_color.color_detect:main',
            'AnomalyHeightRemoval =dofbot_pro_color.AnomalyHeightRemoval:main',
            'shape_recognize = dofbot_pro_color.shape_recognize:main',
            'shape_height = dofbot_pro_color.shape_height:main',
            'color_follow = dofbot_pro_color.color_follow:main'
        ],
    },
)
