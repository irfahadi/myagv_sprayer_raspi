import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'myagv_sprayer_navigation'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deva',
    maintainer_email='info@maorthonet.com',
    description='Map, localization and SLAM for the nursery robot. Path planning itself '
                'lives on the Jetson (sprayer_nav); Nav2 is intentionally not used.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'map_server_node = myagv_sprayer_navigation.map_server_node:main',
            'localization_node = myagv_sprayer_navigation.localization_node:main',
        ],
    },
)
