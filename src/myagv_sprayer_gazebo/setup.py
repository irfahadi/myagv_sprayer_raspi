import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'myagv_sprayer_gazebo'


def model_files():
    """Every file under models/, keeping the directory layout Gazebo expects."""
    out = []
    for root, _dirs, files in os.walk('models'):
        if files:
            out.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'tools'), glob('tools/*.py')),
    ] + model_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deva',
    maintainer_email='info@maorthonet.com',
    description='Gazebo world of the 8 x 6 m nursery room with ArUco tray markers, '
                'plus the scenario obstacle spawner used by the benchmark.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scenario_spawner_node = myagv_sprayer_gazebo.scenario_spawner_node:main',
        ],
    },
)
