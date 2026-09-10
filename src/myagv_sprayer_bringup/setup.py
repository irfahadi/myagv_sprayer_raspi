import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'myagv_sprayer_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deva',
    maintainer_email='info@maorthonet.com',
    description='Bringup (hardware + simulation) for the myAGV precision sprayer on the Raspberry Pi.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_stream_node = myagv_sprayer_bringup.camera_stream_node:main',
            'tray_marker_node = myagv_sprayer_bringup.tray_marker_node:main',
            'robot_health_node = myagv_sprayer_bringup.robot_health_node:main',
            'cmd_vel_watchdog_node = myagv_sprayer_bringup.cmd_vel_watchdog_node:main',
        ],
    },
)
