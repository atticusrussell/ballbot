from setuptools import find_packages, setup

package_name = 'dofbot_pro_KCF'

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
            'KCF_TrackAndGrap = dofbot_pro_KCF.KCF_TrackAndGrap:main',
            'compute_width = dofbot_pro_KCF.compute_width:main',
            'KCF_Tracker = dofbot_pro_KCF.KCF_Tracker:main',
            'ALM_KCF_Tracker = dofbot_pro_KCF.ALM_KCF_Tracker:main'
        ],
    },
)
