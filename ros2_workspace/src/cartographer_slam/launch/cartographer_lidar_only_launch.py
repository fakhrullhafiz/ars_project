"""
cartographer_lidar_only_launch.py
------------------------------------------------------------------------------
Layer 3 stretch: genuinely odometry-free 2D LIDAR SLAM via Cartographer
(cartographer_lidar_only.lua sets use_odometry = false). Built after the
chassis crack made wheel-encoder odometry (the wheel_odometry package)
unreliable -- see CLAUDE.md's Current Status. Unlike the earlier
slam_toolbox_lidar_only.yaml attempt, which fed slam_toolbox a frozen
identity odom->base_link transform and stalled after one scan (slam_toolbox
gates new scans on real odom movement), Cartographer estimates motion itself
from consecutive scans and needs no odom input at all.

Bring up the LIDAR driver FIRST in its own terminal and confirm /scan is
publishing real data before launching this -- same debugging philosophy as
obstacle_avoidance_launch.py, so a scan-data problem and a SLAM problem
aren't debugged as one blob:

    Terminal 1: ros2 launch ydlidar_ros2_driver ydlidar_launch.py
                (activate the lifecycle node if /scan doesn't appear --
                see README/CLAUDE.md)
    Terminal 2: ros2 launch cartographer_slam cartographer_lidar_only_launch.py

View it live: rviz2, Fixed Frame = map, add a Map display on /map and a
LaserScan display on /scan (Reliability Policy: Best Effort, same as
combined_view.rviz already needs -- see CLAUDE.md).

Requires ros-jazzy-cartographer + ros-jazzy-cartographer-ros installed via
apt (not vendored in this repo, unlike ydlidar_ros2_driver).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('cartographer_slam'), 'config')

    # Defaults false so real-hardware runs are unaffected (there, both the
    # LIDAR driver and this node use wall-clock time). Set true ONLY when
    # driving off a Gazebo sim, which publishes simulated time on /clock --
    # otherwise the scan timestamps (sim time) and the node's clock (wall
    # time) disagree by decades and Cartographer's pose extrapolation
    # diverges to garbage (observed: pose jumping to tens of thousands of
    # metres). Usage: ... cartographer_lidar_only_launch.py use_sim_time:=true
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock sim time -- set true only when running against Gazebo.')

    # Which .lua to load. Defaults to the odometry-FREE hardware config, so
    # the real-robot command line is unchanged. Override to
    # cartographer_sim_odom.lua (use_odometry=true) for the sim odometry test.
    config_basename = LaunchConfiguration('configuration_basename')
    declare_config_basename = DeclareLaunchArgument(
        'configuration_basename', default_value='cartographer_lidar_only.lua',
        description='Cartographer .lua config file name (in this package\'s config/).')

    # Topic Cartographer subscribes to for odometry when the chosen config has
    # use_odometry=true. Default 'odom' is a harmless no-op remap for the
    # odometry-free hardware config (which ignores odom entirely). For sim,
    # set to /mecanum_drive_controller/odometry.
    odom_topic = LaunchConfiguration('odom_topic')
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='odom',
        description='nav_msgs/Odometry topic to feed Cartographer (only used if the config sets use_odometry=true).')

    return LaunchDescription([
        declare_use_sim_time,
        declare_config_basename,
        declare_odom_topic,
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('odom', odom_topic)],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', config_basename,
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05'],
        ),
    ])
