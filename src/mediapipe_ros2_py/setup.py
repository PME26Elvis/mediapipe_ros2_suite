# ~/ros2_ws/src/mediapipe_ros2_py/setup.py
import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'mediapipe_ros2_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Mediapipe tasks exposed as ROS 2 nodes',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'mp_node = mediapipe_ros2_py.mp_node:main',
            'gesture_to_turtlesim = mediapipe_ros2_py.gesture_to_turtlesim:main',
        ],
    },
)
