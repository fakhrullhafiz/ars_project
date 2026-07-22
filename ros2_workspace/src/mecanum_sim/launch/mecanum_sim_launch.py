"""
mecanum_sim_launch.py
------------------------------------------------------------------------------
Spawns the simplified mecanum robot (mecanum_robot.urdf.xacro -- real
wheelbase/track/wheel-radius from wheel_odometry_node.py, placeholder
chassis box, see that file's header) into a plain test room
(worlds/mecanum_test_room.sdf) in Gazebo Sim, and bridges its simulated
LIDAR to a real ROS2 /scan topic -- the goal is to test the LIDAR-only
Cartographer SLAM pipeline (cartographer_slam package) against something
that isn't fighting USB brownouts, not to model the real robot precisely.

Brings up, in order:
  1. Gazebo Sim (gz sim) with the test room world.
  2. robot_state_publisher, publishing /robot_description from the xacro
     (with controllers_file baked in for the gz_ros2_control plugin).
  3. Spawns the robot into gz sim from /robot_description.
  4. controller_manager spawners: joint_state_broadcaster, then
     mecanum_drive_controller.
  5. ros_gz_bridge, translating the simulated LIDAR's gz-transport topic
     into a real ROS2 sensor_msgs/LaserScan on /scan.

Once /scan is confirmed live (`ros2 topic echo /scan`), point
cartographer_slam at it exactly as with real hardware:
    ros2 launch cartographer_slam cartographer_lidar_only_launch.py

UNVERIFIED as of writing (first attempt, not yet run):
  - mecanum_drive_controller's exact command topic name/type has changed
    across ros2_controllers versions (older: plain Twist on /cmd_vel;
    newer: TwistStamped on <controller_name>/reference) -- confirm with
    `ros2 topic list` / `ros2 topic info` once this is running before
    trying to drive it, rather than assuming either name is right.
  - The LIDAR's gz-transport topic name (set to 'scan' via the sensor's
    <topic> tag in the xacro) is bridged 1:1 below; if gz sim's actual
    topic ends up scoped/prefixed differently, adjust the bridge argument
    to match (`gz topic -l` after launch will show the real name).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'mecanum_test_room.sdf')
    controllers_file = os.path.join(pkg_share, 'config', 'mecanum_drive_controller.yaml')

    robot_description = {
        # Wrapped as an explicit string ParameterValue -- otherwise launch
        # tries to parse the Command() substitution's URDF/XML output as
        # YAML and fails before robot_state_publisher even starts.
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' controllers_file:=', controllers_file]),
            value_type=str,
        )
    }

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mecanum_robot', '-z', '0.05'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller'],
        output='screen',
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen',
    )

    # The published /scan's header.frame_id is a Gazebo-generated scoped
    # name (confirmed live: "mecanum_robot/base_link/lidar"), not the URDF's
    # "laser_frame" -- laser_frame has no <inertial>, so gz sim's URDF->SDF
    # conversion lumps it into its parent (base_link) as a fixed-joint
    # optimization, and the sensor inherits base_link's scoped name instead.
    # Cartographer looks up a TF for whatever frame_id is actually in the
    # message, so without this bridge its tf_bridge lookup fails forever
    # (confirmed: continuous "does not exist" warnings, not transient).
    # This publishes that missing edge directly, using the real observed
    # name rather than guessing at gz sensor frame-override syntax.
    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.02', '0', '0', '0', '1',
                    'base_link', 'mecanum_robot/base_link/lidar'],
    )

    # Chained via OnProcessExit rather than launched in parallel -- the
    # spawners hitting controller_manager's services before gz_ros2_control
    # has fully finished registering hardware interfaces after entity
    # creation is a well-known race (confirmed here: both controllers'
    # "configure" step failed once, then joint_state_broadcaster's own
    # "load" step failed a second time, non-deterministically). This is
    # the standard fix used by gz_ros2_control_demos for exactly this.
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    load_mecanum_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_mecanum_drive_controller,
        lidar_bridge,
        lidar_frame_bridge,
    ])
