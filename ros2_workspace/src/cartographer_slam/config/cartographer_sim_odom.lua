-- cartographer_sim_odom.lua
-- ----------------------------------------------------------------------------
-- SIM-ONLY variant of cartographer_lidar_only.lua. The ONE meaningful
-- difference is `use_odometry = true` below: Cartographer additionally
-- consumes a nav_msgs/Odometry stream (the mecanum_drive_controller's
-- /mecanum_drive_controller/odometry, remapped to `odom` by the launch file)
-- to SEED each scan-match, instead of relying on scan-to-scan matching alone.
--
-- Why sim-only, not the default: on the real robot the chassis crack makes
-- wheel odometry untrustworthy (see CLAUDE.md), which is the whole reason the
-- hardware path is odometry-FREE (cartographer_lidar_only.lua,
-- use_odometry = false). That constraint does not exist in sim, where the
-- controller's odometry is clean -- so this file exists to measure how much
-- odometry-seeded SLAM improves the fragile translation tracking seen in the
-- odometry-free sim test (see mecanum_sim/README.md Findings). Do NOT point
-- this config at the real robot until its odometry is trustworthy again.
--
-- Everything else is copied verbatim from cartographer_lidar_only.lua so the
-- only variable under test is the odometry input. provide_odom_frame stays
-- true (Cartographer still owns the odom->base_link TF; the controller's own
-- odom TF is disabled via enable_odom_tf:false in
-- mecanum_drive_controller.yaml, so there's no conflicting publisher) --
-- use_odometry consumes the odom TOPIC as a sensor, which is separate from
-- who publishes the odom FRAME.

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
  use_odometry = true,
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

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(45.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2

POSE_GRAPH.optimize_every_n_nodes = 20

return options
