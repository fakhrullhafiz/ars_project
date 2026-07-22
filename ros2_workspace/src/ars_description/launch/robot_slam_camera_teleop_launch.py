"""
robot_slam_camera_teleop_launch.py
------------------------------------------------------------------------------
Same full stack as robot_slam_camera_launch.py -- robot model, LIDAR,
Cartographer SLAM, RealSense (staggered depth-then-RGB, Motion Module off),
DepthCloud, RViz -- EXCEPT it does NOT launch encoder_joint_state_node.

Why: to actually drive the robot under motor power while mapping, you need
a second terminal sending keyboard commands over the Arduino's serial port.
encoder_joint_state_node's own docstring is explicit that only one process
may have that serial port open at a time -- so a separate teleop script
running alongside it would be exactly the untested dual-open it warns
against. teleop_bridge_node.py solves this by doing BOTH jobs (telemetry
-> /joint_states AND keyboard -> drive commands) from one process -- but it
needs raw, exclusive access to its own terminal's stdin for keypresses,
which `ros2 launch` can't give an individual Node() (it multiplexes
several processes into one terminal). So teleop_bridge must run as a plain
`ros2 run` in its own terminal, and this launch file leaves that node out
to avoid a double-serial-open the moment you start it there.

RUN (two terminals):
    Terminal 1: ros2 launch ars_description robot_slam_camera_teleop_launch.py
    Terminal 2: ros2 run ars_description teleop_bridge
                (add --ros-args -p port:=/dev/ttyUSBx if it's not on the
                default /dev/ttyUSB1)

Keys (typed into Terminal 2, no Enter needed): w/a/s/d/q/e drive, x stop,
SPACE emergency-stop, g clear-emergency-stop, Ctrl-C in Terminal 2 to quit
teleop (leaves the rest of the stack in Terminal 1 running).

Everything else -- staggered camera start, placeholder base_link->camera_link
TF, DepthCloud caveats -- is identical to robot_slam_camera_launch.py; see
that file's docstring for the full reasoning.
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
    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_slam'), 'config')

    xacro_file = os.path.join(ars_share, 'urdf', 'ars_robot.urdf.xacro')
    rviz_config = os.path.join(ars_share, 'rviz', 'robot_slam_camera_view.rviz')
    ydlidar_params = os.path.join(ydlidar_share, 'params', 'ydlidar.yaml')

    color_delay = LaunchConfiguration('color_delay')
    declare_color_delay = DeclareLaunchArgument(
        'color_delay', default_value='2.0',
        description='Buffer (s) between starting depth and enabling RGB -- see '
                    'robot_slam_camera_launch.py docstring.')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
    }

    return LaunchDescription([
        declare_color_delay,

        # NOTE: no `port` launch arg here, and no encoder_joint_state_node --
        # run `ros2 run ars_description teleop_bridge` in a separate terminal
        # instead, which owns the Arduino serial port for both telemetry and
        # driving. See this file's docstring.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
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
