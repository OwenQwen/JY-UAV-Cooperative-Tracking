from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'uav_tracking_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JL_tracking',
    maintainer_email='198208059+OwenQwen@users.noreply.github.com',
    description='High-level cooperative tracking mission control for PX4 via ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = uav_tracking_control.mission_node:main',
            'mission_node.py = uav_tracking_control.mission_node:main',
            'target_simulator = uav_tracking_control.target_simulator:main',
            'target_simulator.py = uav_tracking_control.target_simulator:main',
        ],
    },
)
