from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fakhrullhafiz',
    maintainer_email='fakhrulhafiz99@gmail.com',
    description='RPLIDAR-based obstacle detection and stop/go serial bridge to the Arduino motion controller.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = obstacle_avoidance.obstacle_detector_node:main',
        ],
    },
)
