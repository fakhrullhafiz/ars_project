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
-- FIXED 2026-07-23: this was math.rad(0.2) -- 0.2 DEGREES, an almost
-- certain units mistake (math.rad() converts degrees->radians, so this is
-- ~20-100x tighter than the 1-20 deg typical for this setting). That tiny
-- a threshold means nearly every scan's noise-level angular jitter exceeds
-- it, so the motion filter -- which is supposed to skip inserting redundant
-- pose-graph nodes when the robot hasn't really moved -- was effectively
-- disabled, inserting a "new" node on almost every scan. This doesn't
-- directly cause the runaway-pose bug fixed above (that's local tracking,
-- not the pose-graph backend), but it adds needless node density/CPU load
-- for no benefit. 1 deg is a standard, conservative value for this field.
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- REVISED 2026-07-23: the 45 deg / 0.2 m widening below (from Cartographer's
-- defaults of ~20 deg / 0.1 m) fixed the 2026-07-22 hand-rotation tracking
-- loss, but a room-mapping test the same day showed the tradeoff cuts the
-- other way -- while stationary in a long, feature-poor corridor-like scan,
-- the wider search window let the correlative matcher alias onto a wrong
-- but higher-scoring offset, and with no odometry/IMU to catch the mistake,
-- the constant-velocity extrapolator compounded that single bad match every
-- subsequent scan, producing a runaway pose (observed: map coordinates in
-- the hundreds of meters after ~1 minute stationary). A wide search window
-- trades rotation-tracking robustness for aliasing risk in low-feature
-- geometry -- both are real failure modes of odometry-free scan matching,
-- not fixable by search-window size alone.
--
-- Compromise, NOT re-validated on hardware yet: 30 deg / 0.15 m. At the
-- LIDAR's ~3.25 Hz scan rate (~0.31 s between scans), 30 deg still tracks
-- roughly (30 / 0.31) =~ 97 deg/s of rotation -- comfortably above normal
-- hand-turning speed, so the 2026-07-22 fix should still hold -- while
-- cutting the search area (and therefore aliasing opportunity) versus 45
-- deg/0.2 m. If tracking loss on fast turns returns, widen angular back up
-- before touching anything else; if runaway drift returns, narrow further
-- and/or avoid mapping long feature-poor corridors with this odometry-free
-- setup.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15

POSE_GRAPH.optimize_every_n_nodes = 20

return options
