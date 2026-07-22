"""
icp_odometry_launch.py
------------------------------------------------------------------------------
ICP (Iterative Closest Point) scan-matching odometry from the YDLidar's
/scan, using RTAB-Map's icp_odometry node. Estimates the robot's motion by
aligning each new LIDAR scan to the previous one -- no wheel encoders
involved, which is the whole point: the chassis crack makes the encoder-based
wheel_odometry untrustworthy (see CLAUDE.md), so this gives a motion estimate
that does not depend on the (mis-aligned) wheels at all.

Publishes:
  /odom                 nav_msgs/Odometry
  TF  odom -> base_link (the robot's estimated pose)

REQUIRES (run these FIRST, in their own terminals):
  1. The LIDAR driver, which provides BOTH /scan AND the base_link->laser_frame
     static TF that ICP needs to place the scan on the robot:
        ros2 launch ydlidar_ros2_driver ydlidar_launch.py
  2. ros-jazzy-rtabmap-odom must be installed:
        sudo apt install ros-jazzy-rtabmap-odom

RUN:
  ros2 launch lidar_odometry icp_odometry_launch.py
  ros2 launch lidar_odometry icp_odometry_launch.py rviz:=false      # no RViz
  ros2 launch lidar_odometry icp_odometry_launch.py use_sim_time:=true

HOW TO TEST / FINE-TUNE (this is a starting config, NOT tuned to your LIDAR
or environment yet):
  - Set RViz Fixed Frame to "odom". Push the robot by hand; the model should
    translate/rotate in RViz matching the real motion, and /odom should change.
  - If the pose JUMPS or drifts badly: the scan match is losing lock. Try
    raising Icp/MaxCorrespondenceDistance (e.g. 0.15) so more points pair up,
    or lowering Icp/VoxelSize (e.g. 0.03) for a denser, more accurate match
    (slower). ICP odometry is weakest in long, featureless corridors (few
    distinct shapes to lock onto) -- same limitation as the cartographer test.
  - See every knob at https://github.com/introlab/rtabmap (Icp/* and Odom/*).
    All rtabmap parameter VALUES must be strings, even the numeric ones.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('lidar_odometry'), 'rviz', 'icp_odom.rviz'])

    icp_params = {
        # Frames. icp_odometry computes the pose of `frame_id` in
        # `odom_frame_id` and publishes that as the odom->base_link TF.
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'wait_for_transform': 0.2,
        'approx_sync': False,          # scan-only: use exact time sync
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        # ydlidar_ros2_driver publishes /scan as Best Effort (sensor data
        # QoS). rtabmap nodes default their scan subscriber to Reliable,
        # which is incompatible -- the subscription connects but zero
        # messages arrive (the same class of bug CLAUDE.md documents for
        # RViz's LaserScan display; see ydlidar_ros2_driver's own launch
        # warning: "requesting incompatible QoS ... RELIABILITY_QOS_POLICY").
        # 1 = rtabmap's "sensor data" (Best Effort) QoS preset.
        'qos_scan': 1,

        # --- ICP tuning knobs (rtabmap: ALL values are strings) ---
        'Reg/Strategy': '1',           # 1 = ICP (vs 0 visual, 2 visual+ICP)
        'Reg/Force3DoF': 'true',       # constrain to a flat floor: x, y, yaw
                                       # only. ESSENTIAL for a 2D-LIDAR ground
                                       # robot, or roll/pitch noise corrupts it.
        'Icp/VoxelSize': '0.05',       # downsample scan to 5 cm before matching.
                                       # smaller = more accurate but slower.
        'Icp/MaxCorrespondenceDistance': '0.1',  # max gap to pair points across
                                       # scans. RAISE if odom loses lock/jumps;
                                       # lower to reject outliers.
        'Icp/Iterations': '10',
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlane': 'false',   # point-to-point: 2D scans are too sparse
                                       # for reliable surface normals.
        'Icp/CorrespondenceRatio': '0.2',  # min fraction of points that must
                                       # match to accept the transform; below
                                       # this the frame is treated as lost.
        'Odom/Strategy': '0',          # 0 = Frame-to-Map (more robust drift-wise)
        'Odom/GuessMotion': 'true',    # seed each match with the last motion
        'Odom/ResetCountdown': '1',    # auto-reset if lost, so it recovers
                                       # instead of freezing at a bad pose.
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (true only in Gazebo sim, not real hardware).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Also open RViz preconfigured to view /odom + /scan.'),

        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[icp_params],
            remappings=[('scan', '/scan')],
        ),

        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2_icp_odom',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            output='screen',
        ),
    ])
