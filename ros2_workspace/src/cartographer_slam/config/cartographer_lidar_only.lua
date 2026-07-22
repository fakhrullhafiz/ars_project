-- cartographer_lidar_only.lua
-- ----------------------------------------------------------------------------
-- Minimal 2D LIDAR-only Cartographer config. The key line is
-- `use_odometry = false` below -- Cartographer's own trajectory builder
-- estimates motion by matching consecutive LIDAR scans against each other,
-- so it needs no /odom input at all (wheel-encoder-based or otherwise).
-- `provide_odom_frame = true` makes Cartographer publish its own internal
-- odom frame, since nothing else on this robot is providing one.
--
-- tracking_frame is base_link, relying on ydlidar_launch.py's existing
-- static base_link -> laser_frame transform to place the LIDAR. No other
-- sensor frames are referenced here, so this doesn't depend on the
-- unmeasured camera placeholder transform noted in CLAUDE.md.
--
-- NOT YET VALIDATED ON HARDWARE -- these are standard/conservative starting
-- values (matching common single-LIDAR Cartographer configs), not tuned
-- against this specific robot or LIDAR. Expect to revisit min_range/max_range
-- and motion_filter thresholds after a real test drive.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- No IMU on this robot (see CLAUDE.md hardware facts) -- scan matching alone
-- has to carry the full motion estimate, so online correlative scan matching
-- is on to help it find larger frame-to-frame offsets, not just refine a
-- near-correct prior.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0  -- matches the LIDAR range ceiling used in slam_toolbox_lidar_only.yaml
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- Widened from Cartographer's defaults (angular 20 deg, linear 0.1 m) after a
-- real test (2026-07-22): rotating the robot by hand caused the map to "lose"
-- the rotation and snap back once turning stopped. Root cause is the LIDAR's
-- slow ~3.25 Hz scan rate (see CLAUDE.md) -- with no odometry/IMU to seed
-- each match, any rotation faster than roughly (angular_search_window /
-- time_between_scans) exceeds what the correlative matcher searches, so it
-- loses tracking. Wider windows search more per scan (more CPU) and given
-- this used to be the default, hand-rotating a chassis is very easy to move
-- faster than 65 deg/s -- deliberately generous, not just nudged.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(45.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2

POSE_GRAPH.optimize_every_n_nodes = 20

return options
