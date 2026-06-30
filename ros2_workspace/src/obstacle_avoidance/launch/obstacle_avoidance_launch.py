"""
obstacle_avoidance_launch.py
------------------------------------------------------------------------------
Launches the obstacle_detector node alone.

This intentionally does NOT also launch the RPLIDAR driver (rplidar_ros) —
keep those as two separate launch steps while debugging, so you always know
whether a problem is in the LIDAR driver or in this package's logic:

    Terminal 1: ros2 launch rplidar_ros rplidar_a1_launch.py
    Terminal 2: ros2 launch obstacle_avoidance obstacle_avoidance_launch.py

Once both are confirmed working independently, it's reasonable to combine
them into one launch file — left as a deliberate later step, not done here.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_avoidance',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
        ),
    ])
