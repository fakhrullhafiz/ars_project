"""
obstacle_avoidance_with_camera_launch.py
------------------------------------------------------------------------------
Adds the RealSense D455's depth image as a second collision-detection
source, alongside the LIDAR. Launches:
  1. realsense2_camera_node, depth stream only (color/infra disabled --
     collision detection only needs depth, and this hub has a real history
     of USB brownouts under load, see CLAUDE.md, so there's no reason to
     make it carry streams this feature doesn't use).
  2. depthimage_to_laserscan, converting that depth image into a
     sensor_msgs/LaserScan on /camera_scan -- same message type the LIDAR
     publishes on /scan, so obstacle_detector_node.py's existing forward-
     cone check works on it unchanged (see that node's docstring).
  3. obstacle_detector (unmodified entry point) -- this launch file does NOT
     bring up the LIDAR driver; run ydlidar_ros2_driver separately first,
     same as obstacle_avoidance_launch.py, so a LIDAR-only run still works
     exactly as before and this is purely additive.

No TF chain (base_link -> camera_link) is required for this feature, unlike
the CLAUDE.md-documented camera/LIDAR point-cloud overlay work -- this node
reads ranges/angles straight out of each LaserScan message in that message's
own frame, the same way it already does for /scan. The unmeasured placeholder
transform noted in CLAUDE.md is irrelevant here.

UNTESTED as of this writing -- the D455 was not physically connected when
this was written. Before trusting it:
  - Confirm the actual published topic names with `ros2 topic list` once
    the camera is connected and this is launched; the remappings below
    assume realsense2_camera's current default camera_name/camera_namespace
    ('camera'/'camera'), giving /camera/camera/depth/image_rect_raw --
    this nesting has changed across realsense2_camera versions before, so
    don't assume it's right without checking.
  - Confirm output_frame below ('camera_depth_optical_frame') matches what
    the driver actually publishes as the depth image's frame_id.
  - This shares the Arduino's serial port with the LIDAR-only obstacle
    path through the same obstacle_detector node/connection -- fine, since
    it's the same node, not two competing connections (see that node's
    docstring for why a second connection is avoided).

    Terminal 1: ros2 launch ydlidar_ros2_driver ydlidar_launch.py
    Terminal 2: ros2 launch obstacle_avoidance obstacle_avoidance_with_camera_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
                # IMU wasn't actually off before -- this Node()'s own default
                # differs from rs_launch.py's (which defaults these to
                # false), so accel/gyro were streaming unnecessarily,
                # adding extra USB isochronous traffic for data this
                # feature never uses.
                'enable_gyro': False,
                'enable_accel': False,
                # Lowest confirmed-valid depth profile for this D455 (see
                # `rs-enumerate-devices` output, 2026-07-22) -- cuts frame
                # data from ~9.2 MB/s (640x480@15) to ~1.3 MB/s. CLAUDE.md's
                # 2026-07-21 test found this same profile still corrupted on
                # the old shared external hub, run alongside a color stream
                # -- this is a genuinely different test: camera is now on a
                # direct SBC port, alone (no color/IMU), so worth confirming
                # rather than assuming the old result still applies.
                'depth_module.depth_profile': '480,270,5',
            }],
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/camera_scan'),
            ],
            parameters=[{
                'scan_time': 0.2,  # matches the 5 Hz depth_module.depth_profile above
                'range_min': 0.20,
                'range_max': 4.0,
                'scan_height': 10,
                'output_frame': 'camera_depth_optical_frame',
            }],
        ),
        Node(
            package='obstacle_avoidance',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
        ),
    ])
