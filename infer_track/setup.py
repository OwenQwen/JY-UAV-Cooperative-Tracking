import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'infer_track'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JL_tracking',
    maintainer_email='198208059+OwenQwen@users.noreply.github.com',
    description='ROS 2 target recognition, tracking, and mission management.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'udp_yolo = infer_track.udp_yolo:main',
            'recognition_node = infer_track.recongnition_node:main',
            'sim_recognition_node = infer_track.sim_recognition_node:main',
            'referee_udp_node = infer_track.referee_udp_node:main',
            'mission_manager = infer_track.mission_manager:main',
            'tracking_node = infer_track.tracking_node:main',
            'coordinate_transform = '
            'infer_track.ROS2_coordinate_transform:main',
            'keyboard_teleop = infer_track.keyboard_teleop:main',
        ],
    },
)
