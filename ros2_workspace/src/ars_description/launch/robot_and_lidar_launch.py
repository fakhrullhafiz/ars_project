"""
robot_and_lidar_launch.py
------------------------------------------------------------------------------
Combined RViz view: the live wheeled robot model (display_launch.py's thing --
wheels driven by real encoder telemetry) PLUS the real YDLidar's /scan, with
NO camera/RealSense in the mix. For when the RealSense has a connectivity
problem and you just want to see the robot + LIDAR working.

Brings up:
  1. robot_state_publisher on ars_robot.urdf.xacro (wheel TFs from encoders,
     AND the base_link->laser_frame static TF -- it's baked into the URDF's
     laser_joint, so we do NOT also launch ydlidar_launch.py's own separate
     static_transform_publisher for the same transform; running both would
     just be a redundant duplicate-named node, like the stale ones cleaned up
     earlier tonight).
  2. encoder_joint_state_node (Arduino "E,..." telemetry -> /joint_states).
  3. The YDLidar driver node directly (same node/params as
     ydlidar_ros2_driver's own ydlidar_launch.py, minus its static transform).
  4. RViz with robot_and_lidar_view.rviz (RobotModel + TF + Grid + LaserScan).

REQUIRES: main_robot.ino on the Arduino (streaming "E,...") and the YDLidar
plugged in. Same ttyUSB caveat as display_launch.py -- Arduino/LIDAR port
numbers can swap; override with `port:=`.

The YDLidar driver is a ROS2 LIFECYCLE node. If /scan doesn't show up within
a few seconds, activate it by hand (see CLAUDE.md's Commands section):
    ros2 lifecycle set /ydlidar_ros2_driver_node configure
    ros2 lifecycle set /ydlidar_ros2_driver_node activate

RUN:
    ros2 launch ars_description robot_and_lidar_launch.py
    ros2 launch ars_description robot_and_lidar_launch.py port:=/dev/ttyUSB0
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

    xacro_file = os.path.join(ars_share, 'urdf', 'ars_robot.urdf.xacro')
    rviz_config = os.path.join(ars_share, 'rviz', 'robot_and_lidar_view.rviz')
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
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
