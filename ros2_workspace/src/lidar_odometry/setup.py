from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tsaqif',
    maintainer_email='mohdtsaqif@gmail.com',
    description='ICP (scan-matching) LIDAR odometry via RTAB-Map icp_odometry '
                '(odometry-free of the cracked wheel encoders, see CLAUDE.md).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
