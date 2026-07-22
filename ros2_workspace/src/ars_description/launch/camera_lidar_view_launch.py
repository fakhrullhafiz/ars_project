"""
camera_lidar_view_launch.py
------------------------------------------------------------------------------
DEMO LAUNCH #2 of 2 -- the "look at our sensors" demo. Shows the RealSense
D455 depth camera (depth image, RGB image, and a live 3D DepthCloud) together
with the YDLidar's laser scan in ONE RViz window. That's it: no SLAM, no map,
no Arduino, no driving.

Use this when you just want to demonstrate the two sensors working together.
It is deliberately lighter than the mapping launch (no Cartographer, no serial
port opened) so it comes up fast and has nothing to steer -- a single command,
nothing else to run.

  (The OTHER demo launch is robot_slam_camera_teleop_launch.py -- the mapping
   demo: LIDAR + Cartographer SLAM + camera, driven with teleop_bridge in a
   second terminal. Use that one to build/save a map; use THIS one to show the
   sensors.)

WHAT IT STARTS:
  1. The YDLidar driver (lifecycle node) -> /scan.
  2. realsense2_camera_node -- STEREO (depth) first, RGB enabled ~2 s later
     via a runtime param set, Motion Module (IMU) OFF. Identical staggered
     start to the mapping launch (this D455 + SBC can't bring up all the
     streams at once -- see CLAUDE.md).
  3. Two static transforms so both sensors share one 3D view:
       base_link -> laser_frame   (0, 0, 0.02, matches the URDF/real mount)
       base_link -> camera_link   (PLACEHOLDER 0.10, 0, 0.05 -- NOT measured;
                                    the camera cloud is only as well-placed as
                                    this guess. Fine for a live demo, not
                                    calibrated geometry -- see CLAUDE.md.)
     (camera_link -> the depth/color optical frames is published by the
      realsense driver itself, so the chain is complete.)
  4. RViz with camera_lidar_view.rviz (Fixed Frame = base_link).

REQUIRES: the YDLidar plugged in, the D455 plugged in (ideally its own USB
port, away from other devices -- see CLAUDE.md's USB findings), and
ros-jazzy-realsense2-camera. No Arduino needed.

The YDLidar driver is a lifecycle node; if /scan doesn't appear in a few
seconds, activate it by hand:
    ros2 lifecycle set /ydlidar_ros2_driver_node activate

RUN:
    ros2 launch ars_description camera_lidar_view_launch.py
    ros2 launch ars_description camera_lidar_view_launch.py color_delay:=3.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    ars_share = get_package_share_directory('ars_description')
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')

    rviz_config = os.path.join(ars_share, 'rviz', 'camera_lidar_view.rviz')
    ydlidar_params = os.path.join(ydlidar_share, 'params', 'ydlidar.yaml')

    color_delay = LaunchConfiguration('color_delay')
    declare_color_delay = DeclareLaunchArgument(
        'color_delay', default_value='2.0',
        description='Buffer (s) between starting depth and enabling RGB -- see '
                    'robot_slam_camera_launch.py docstring for why the camera '
                    'streams must start staggered on this hardware.')

    return LaunchDescription([
        declare_color_delay,

        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[ydlidar_params],
            namespace='/',
        ),

        # --- RealSense D455: STEREO (depth) now, RGB after the buffer ---------
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'enable_depth': True,
                'enable_color': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_sync': False,
                'pointcloud.enable': False,
                'enable_gyro': False,
                'enable_accel': False,
            }],
        ),
        TimerAction(
            period=color_delay,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/camera/camera',
                         'enable_color', 'true'],
                    output='screen',
                ),
            ],
        ),

        # --- static frames so both sensors share one 3D view -----------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.02',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_placeholder',
            arguments=[
                '--x', '0.10', '--y', '0.0', '--z', '0.05',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
            ],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
