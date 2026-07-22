# mecanum_sim

Gazebo Sim model of the robot for testing the LIDAR-only Cartographer SLAM
pipeline (`cartographer_slam` package) in a repeatable environment, decoupled
from the real hardware's USB/power problems.

**Real dimensions**, from the actual codebase (not guessed): wheelbase
`0.142m`, track width `0.195m`, wheel diameter `97mm` (from
`wheel_odometry_node.py` + `encoder_test.ino`). **Placeholder**: the chassis
body is a plain box sized to fit the wheels — not measured against the real
chassis (see `urdf/mecanum_robot.urdf.xacro` header). Good enough to test
whether SLAM works; not a faithful physical replica.

## Run it (each in its own terminal, source first)

**Terminal 1 — sim + robot + LIDAR bridge:**
```bash
cd ~/ars_project/ros2_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mecanum_sim mecanum_sim_launch.py
```
Wait for `Configured and activated mecanum_drive_controller`. A Gazebo window
opens showing the room. Confirm the LIDAR is publishing: `ros2 topic hz /scan`
(~10 Hz).

**Terminal 2 — Cartographer SLAM (NOTE `use_sim_time:=true`):**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ars_project/ros2_workspace/install/setup.bash
ros2 launch cartographer_slam cartographer_lidar_only_launch.py use_sim_time:=true
```
`use_sim_time:=true` is **mandatory in sim** — without it Cartographer mixes
wall-clock and sim time and its pose diverges to garbage. Omit it for real
hardware.

**Terminal 3 — RViz (watch the map build):**
```bash
source /opt/ros/jazzy/setup.bash
rviz2 -d ~/ars_project/ros2_workspace/rviz/slam_test.rviz
```
Set **Displays → Global Options → Fixed Frame = map**, and if the map/scan
don't show, check the same `Reliability Policy: Best Effort` note as
`combined_view.rviz`.

**Terminal 4 — drive the robot** (the controller wants `TwistStamped`):
```bash
source /opt/ros/jazzy/setup.bash
# forward at 0.15 m/s:
ros2 topic pub --rate 20 /mecanum_drive_controller/reference \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.15}}}'
# strafe sideways (mecanum!): set linear.y instead of linear.x
# rotate in place: set angular.z
```
Drive **slowly** and avoid ramming walls — see Findings below.

## Findings (2026-07-22, first bring-up)

- **Pipeline works end to end**: robot spawns with real wheel geometry,
  mecanum drive control works (verified: robot physically moved ~2.7m
  forward and rotated on command, via Gazebo ground-truth pose), simulated
  LIDAR publishes a clean 360-point `/scan`, and Cartographer builds an
  occupancy map from it.
- **Rotation tracking is accurate** — commanded a 90° in-place turn, and
  Cartographer's pose estimate read 90.6°.
- **Translation tracking is fragile under fast motion / collisions.** In the
  first test the robot was driven hard into a wall and got stuck (wheels
  slipping — the controller's own wheel odometry then read a nonsense 35m
  while ground truth was 2.7m), and Cartographer under-tracked the forward
  motion badly. This mirrors the real-hardware finding (CLAUDE.md): pure
  odometry-free scan-matching is fragile, especially approaching large flat
  featureless walls, with no odometry/IMU to seed each match.
- **The obvious next improvement, available in sim (and on a repaired real
  robot):** feed Cartographer the wheel odometry as a *seed*
  (`/mecanum_drive_controller/odometry` exists here) by setting
  `use_odometry = true` in `cartographer_lidar_only.lua`. We went
  odometry-free on the real robot only because the chassis crack made its
  odometry untrustworthy — that constraint doesn't apply in sim, so this is
  a good way to see how much odometry-seeded SLAM improves things.
