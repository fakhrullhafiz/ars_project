"""
robot_and_slam_launch.py
------------------------------------------------------------------------------
The live wheeled robot model PLUS continuously-updating Cartographer SLAM:
/map keeps growing and map->odom->base_link keeps updating live as you
physically move/rotate the robot, matching what was seen when a prior
session ran Cartographer -- that behavior comes from the LIDAR alone
(cartographer_lidar_only.lua's use_odometry=false), NOT from the RealSense
camera, which is not part of this pipeline at all. No camera anywhere here.

Brings up:
  1. robot_state_publisher on ars_robot.urdf.xacro (wheel TFs from encoders,
     plus the base_link->laser_frame static TF baked into the URDF).
  2. encoder_joint_state_node (Arduino "E,..." telemetry -> /joint_states).
  3. The YDLidar driver node directly (see robot_and_lidar_launch.py's
     docstring for why we don't also launch its bundled static transform).
  4. cartographer_node + cartographer_occupancy_grid_node (odometry-free
     LIDAR SLAM -- same nodes as cartographer_slam's own launch file).
     tracking_frame/published_frame = base_link, so this composes cleanly
     with the URDF above: Cartographer owns map->odom->base_link, the URDF
     owns base_link->{wheels,laser_frame}. No TF ownership conflict.
  5. RViz with robot_and_slam_view.rviz (Map + RobotModel + TF + LaserScan,
     Fixed Frame = map -- NOT base_link, since the whole point is to watch
     the robot move THROUGH a growing map, not sit still at the origin).

REQUIRES: main_robot.ino on the Arduino (streaming "E,...") and the YDLidar
plugged in, plus ros-jazzy-cartographer + ros-jazzy-cartographer-ros
(already installed this session).

The YDLidar driver is a ROS2 LIFECYCLE node; if /scan doesn't appear within
a few seconds, activate it by hand:
    ros2 lifecycle set /ydlidar_ros2_driver_node configure
    ros2 lifecycle set /ydlidar_ros2_driver_node activate
(It has been auto-activating on its own the last few runs -- if that quirk
persists you likely won't need this at all.)

RUN:
    ros2 launch ars_description robot_and_slam_launch.py
    ros2 launch ars_description robot_and_slam_launch.py port:=/dev/ttyUSB0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ars_share = get_package_share_directory('ars_description')
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_slam'), 'config')

    xacro_file = os.path.join(ars_share, 'urdf', 'ars_robot.urdf.xacro')
    rviz_config = os.path.join(ars_share, 'rviz', 'robot_and_slam_view.rviz')
    ydlidar_params = os.path.join(ydlidar_share, 'params', 'ydlidar.yaml')

    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB1',
        description='Arduino serial port (CH340). LIDAR/Arduino ttyUSB numbers can swap; '
                    'see ars_description/udev/99-ars-arduino.rules for a stable /dev/arduino.')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
    }

    return LaunchDescription([
        declare_port,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='ars_description',
            executable='encoder_joint_state',
            output='screen',
            parameters=[{'port': port}],
        ),
        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[ydlidar_params],
            namespace='/',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', 'cartographer_lidar_only.lua',
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
