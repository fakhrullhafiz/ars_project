"""
display_launch.py
------------------------------------------------------------------------------
Live RViz view of the real robot: shows the chassis + 4 wheels + LIDAR mount,
with the wheels driven by the real encoders. Turn a wheel by hand and it
moves in RViz in real time (and its TF axes rotate).

Brings up:
  1. robot_state_publisher on ars_robot.urdf.xacro (turns /joint_states into
     the wheel TFs).
  2. encoder_joint_state_node, reading the Arduino's "E,..." telemetry into
     /joint_states.
  3. RViz with robot_view.rviz (RobotModel + TF + Grid).

REQUIRES: main_robot.ino (or any sketch emitting the "E,fl,fr,rl,rr,millis"
line) loaded on the Arduino. It stays in IDLE and only streams telemetry --
it will not drive unless you send it a command.

Arduino port: defaults to /dev/ttyUSB1 (current wiring -- the LIDAR is
ttyUSB0). These CAN swap across reconnects; override with
`port:=/dev/ttyUSB0` etc., or install the 99-ars-arduino.rules udev rule for
a stable /dev/arduino and use `port:=/dev/arduino`.

    ros2 launch ars_description display_launch.py
    ros2 launch ars_description display_launch.py port:=/dev/arduino
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('ars_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ars_robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')

    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB1',
        description='Arduino serial port (CH340). LIDAR/Arduino ttyUSB numbers can swap; '
                    'see 99-ars-arduino.rules for a stable /dev/arduino.')

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
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
