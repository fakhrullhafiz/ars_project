"""
robot_slam_camera_launch.py
------------------------------------------------------------------------------
The full sensor stack in ONE RViz window: the live wheeled robot model +
Arduino encoders + YDLidar + odometry-free Cartographer SLAM (the current
"latest" SLAM method) + the RealSense D455 depth (stereo module) and RGB
streams. Everything robot_and_slam_launch.py brings up, PLUS the camera.

  1-6. Identical to robot_and_slam_launch.py: robot_state_publisher,
       encoder_joint_state (Arduino), the YDLidar lifecycle node,
       cartographer_node + cartographer_occupancy_grid_node, and RViz --
       except RViz loads robot_slam_camera_view.rviz (adds camera displays).
  7.   realsense2_camera_node -- STEREO MODULE (depth) + RGB only, Motion
       Module (IMU) permanently OFF.
  8.   A staggered start for the camera streams (see below).
  9.   A base_link -> camera_link static transform so the camera's frames
       connect to the robot in RViz.

WHY THE STAGGERED START (tested, confirmed on this exact D455 + SBC):
  This hardware CANNOT bring up all three RealSense modules at once, and
  even depth + RGB must not start simultaneously -- doing so triggers the
  disconnect/reconnect + "Motion Module force pause" failure documented in
  CLAUDE.md. The confirmed-working sequence is:
      1. start depth (stereo module)   <- node launches with enable_depth=true
      2. buffer 2 s                     <- TimerAction(color_delay)
      3. start RGB                      <- `ros2 param set ... enable_color true`
  So this file launches the node with enable_color=FALSE, then enables color
  at runtime after `color_delay` seconds. enable_gyro/enable_accel stay
  FALSE the whole time -- the Motion Module is deliberately never started
  (per our testing it's the module that can't coexist with the other two).

  NOTE on the `enable_accel true` / `enable_gyro true` command from the
  forum: that ENABLES the IMU (Motion Module) -- the opposite of what we
  want. We keep both false. `enable_color` is the parameter that is
  runtime-toggled here, and it IS dynamically settable on the ROS2 wrapper.

CAMERA <-> ROBOT TRANSFORM IS A PLACEHOLDER, NOT A MEASUREMENT.
  base_link -> camera_link below uses the same unmeasured guess CLAUDE.md
  documents (x=0.10, y=0, z=0.05). The camera data will render in roughly
  the right place, but it is NOT calibrated against the real chassis -- do
  not read the combined view as accurate geometry until Haikal's CAD gives
  a real mount pose. Cartographer/SLAM does NOT use the camera at all (it's
  LIDAR-only), so this placeholder cannot corrupt the map -- it only affects
  where the camera imagery appears in the 3D view.

REQUIRES: main_robot.ino on the Arduino (streaming "E,..."), the YDLidar
plugged in, the D455 plugged in (ideally on its own USB3 / separate port --
see CLAUDE.md's USB findings), ros-jazzy-cartographer(+-ros), and
ros-jazzy-realsense2-camera.

The YDLidar driver is a lifecycle node; if /scan doesn't appear in a few
seconds, activate it by hand (see robot_and_slam_launch.py's docstring).

RUN:
    ros2 launch ars_description robot_slam_camera_launch.py
    ros2 launch ars_description robot_slam_camera_launch.py port:=/dev/ttyUSB0
    ros2 launch ars_description robot_slam_camera_launch.py color_delay:=3.0
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

    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB1',
        description='Arduino serial port (CH340). LIDAR/Arduino ttyUSB numbers can swap; '
                    'see ars_description/udev/99-ars-arduino.rules for a stable /dev/arduino.')

    # Seconds to wait after the depth stream (stereo module) comes up before
    # enabling the RGB stream. 2.0 is our tested-good buffer; bump it if the
    # camera ever still hiccups when color turns on.
    color_delay = LaunchConfiguration('color_delay')
    declare_color_delay = DeclareLaunchArgument(
        'color_delay', default_value='2.0',
        description='Buffer (s) between starting depth and enabling RGB -- see docstring.')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
    }

    return LaunchDescription([
        declare_port,
        declare_color_delay,

        # --- same stack as robot_and_slam_launch.py ---------------------------
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

        # --- RealSense D455: STEREO (depth) now, RGB after the buffer ---------
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'enable_depth': True,      # stereo module -- step 1, up at launch
                'enable_color': False,     # RGB -- enabled later, see TimerAction
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_sync': False,
                'pointcloud.enable': False,
                'enable_gyro': False,      # Motion Module OFF, permanently
                'enable_accel': False,     # Motion Module OFF, permanently
            }],
        ),
        # Step 2 + 3: wait color_delay seconds, then turn RGB on at runtime.
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

        # --- PLACEHOLDER camera pose (see docstring -- NOT measured) ----------
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

        # --- one RViz window for everything ----------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
