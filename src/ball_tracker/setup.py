from setuptools import find_packages, setup

package_name = 'ball_tracker'

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
    maintainer='upsurge',
    maintainer_email='cp3873@nyu.edu',
    description='Ball Tracker Node using OpenCV, ROS2 and InferencePipeline for table tennis ball detection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_tracker = ball_tracker.ball_tracker:main',
            'ros_inference_tracker_node = ball_tracker.ros_inference_tracker_node:main',
            'ping_pong_detector = ball_tracker.ping_pong_detector:main'
        ],
    },
)
