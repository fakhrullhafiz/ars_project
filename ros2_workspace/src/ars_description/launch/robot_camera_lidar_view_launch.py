"""
robot_camera_lidar_view_launch.py
------------------------------------------------------------------------------
DEMO LAUNCH: robot model (with LIVE wheel axes from encoder telemetry) +
LIDAR + RealSense depth/RGB/DepthCloud, all in one RViz window. No
Cartographer, no /map, no serial-owned teleop -- this is a "look at
everything the robot senses and how it's built" view, not a driving or
mapping session.

Difference from the other two demo launches:
  - camera_lidar_view_launch.py: camera + LIDAR only, NO robot model/wheel
    axes (Fixed Frame = base_link, but base_link itself is just a static
    frame there -- no URDF, no wheels).
  - robot_slam_camera_teleop_launch.py: everything THIS launch has, PLUS
    Cartographer SLAM (Fixed Frame = map) -- heavier, and expects
    teleop_bridge running separately in its own terminal to drive.
  - THIS launch: robot model + wheel axes + LIDAR + camera, nothing else.
    Fixed Frame = base_link (no map frame exists here, there's no SLAM).

Brings up:
  1. robot_state_publisher on ars_robot.urdf.xacro (chassis + 4 wheels +
     LIDAR mount).
  2. encoder_joint_state_node -- reads the Arduino's "E,..." telemetry and
     publishes /joint_states, so the 4 wheel TFs (and their axes, via the
     TF display's "Show Axes") are LIVE and spin if a wheel is turned by
     hand. This does NOT drive the robot and does not need a second
     terminal (unlike teleop_bridge, it needs no interactive keyboard, so
     it's safe to bundle in a launch file).
  3. The YDLidar driver (lifecycle node) -> /scan.
  4. RealSense D455 -- STEREO (depth) first, RGB enabled ~2s later via a
     runtime param set, Motion Module (accel/gyro) OFF -- this hardware
     cannot run all three at once (see CLAUDE.md); depth+RGB is the
     confirmed-working pair.
  5. A base_link -> camera_link static transform -- UNMEASURED PLACEHOLDER
     (x=0.10, y=0, z=0.05), same guess used everywhere else in this repo.
     Fine for a demo view, not calibrated geometry.
  6. RViz with robot_camera_lidar_view.rviz (RobotModel + TF with axes
     shown + LaserScan + RealSense Color/Depth images + DepthCloud,
     Fixed Frame = base_link).

REQUIRES: main_robot.ino running (for encoder telemetry), the YDLidar
plugged in, the D455 plugged in, ros-jazzy-realsense2-camera. Only ONE
process may own the Arduino's serial port -- this launch's
encoder_joint_state_node is that owner; do NOT also run teleop_bridge or
obstacle_detector_node at the same time (both open their own serial
connection -- see those nodes' docstrings).

The YDLidar driver is a lifecycle node; if /scan doesn't appear in a few
seconds, activate it by hand:
    ros2 lifecycle set /ydlidar_ros2_driver_node activate

RUN:
    ros2 launch ars_description robot_camera_lidar_view_launch.py
    ros2 launch ars_description robot_camera_lidar_view_launch.py port:=/dev/ttyUSB0
    ros2 launch ars_description robot_camera_lidar_view_launch.py color_delay:=3.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ars_share = get_package_share_directory('ars_description')
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')

    xacro_file = os.path.join(ars_share, 'urdf', 'ars_robot.urdf.xacro')
    rviz_config = os.path.join(ars_share, 'rviz', 'robot_camera_lidar_view.rviz')
    ydlidar_params = os.path.join(ydlidar_share, 'params', 'ydlidar.yaml')

    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/arduino',
        description='Arduino serial port. Defaults to the stable udev symlink '
                    '(see ars_description/udev/99-ars-arduino.rules) rather than '
                    'a raw ttyUSBx number, since the LIDAR/Arduino ttyUSB numbers '
                    'can swap across reconnects.')

    color_delay = LaunchConfiguration('color_delay')
    declare_color_delay = DeclareLaunchArgument(
        'color_delay', default_value='2.0',
        description='Buffer (s) between starting depth and enabling RGB -- this '
                    'hardware cannot bring up depth+RGB simultaneously, see CLAUDE.md.')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
    }

    return LaunchDescription([
        declare_port,
        declare_color_delay,

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
                'enable_gyro': False,   # Motion Module OFF -- see docstring
                'enable_accel': False,  # only 2 of 3 modules can run at once
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

        # --- PLACEHOLDER camera pose (NOT measured, see docstring) ------------
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
