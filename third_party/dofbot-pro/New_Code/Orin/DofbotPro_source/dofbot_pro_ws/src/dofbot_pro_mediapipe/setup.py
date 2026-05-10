from setuptools import find_packages, setup

package_name = 'dofbot_pro_mediapipe'

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
            '01_HandDetector = dofbot_pro_mediapipe.01_HandDetector:main',
            '02_PoseDetector = dofbot_pro_mediapipe.02_PoseDetector:main',
            '03_Holistic = dofbot_pro_mediapipe.03_Holistic:main',
            '04_FaceMesh = dofbot_pro_mediapipe.04_FaceMesh:main',
            '05_FaceDetection = dofbot_pro_mediapipe.05_FaceDetection:main',
            '06_FaceLandmarks = dofbot_pro_mediapipe.06_FaceLandmarks:main',
            '07_Objectron = dofbot_pro_mediapipe.07_Objectron:main',
            '08_VirtualPaint = dofbot_pro_mediapipe.08_VirtualPaint:main',
            '09_HandCtrl = dofbot_pro_mediapipe.09_HandCtrl:main',
            '10_GestureRecognition = dofbot_pro_mediapipe.10_GestureRecognition:main',
            '11_GestureAction = dofbot_pro_mediapipe.11_GestureAction:main',
            '12_PoseArm = dofbot_pro_mediapipe.12_PoseArm:main',
            '13_FindHand = dofbot_pro_mediapipe.13_FindHand:main',
            '14_HandFollow = dofbot_pro_mediapipe.14_HandFollow:main',
            '15_FingerTrajectory = dofbot_pro_mediapipe.15_FingerTrajectory:main',
            '16_FingerAction = dofbot_pro_mediapipe.16_FingerAction:main',
            '17_GestureGrasp = dofbot_pro_mediapipe.17_GestureGrasp:main',
            '18_HandCtrlArm = dofbot_pro_mediapipe.18_HandCtrlArm:main',
        ],
    },
)
