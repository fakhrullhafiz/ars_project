#!/usr/bin/env python3
"""
obstacle_detector_node.py
------------------------------------------------------------------------------
Layer 2 stretch goal node.

Subscribes to /scan (sensor_msgs/LaserScan), published by whichever LIDAR
driver is running (currently ydlidar_ros2_driver, vendored in this
workspace at ros2_workspace/src/ydlidar_ros2_driver -- see its launch
file). This node has no brand-specific dependency on the driver; it only
needs a standard /scan topic. Does NOT talk to the LIDAR's serial port
directly — that's the driver's job, brought up separately.

Bring up the LIDAR driver first and confirm `ros2 topic echo /scan` shows
real data BEFORE running this node — debug the sensor in isolation before
adding detection logic on top of it.

This node looks at the scan readings in front of the robot (a configurable
angular window centered on 0 degrees) and checks whether anything is closer
than STOP_DISTANCE_M. If so, it sends a single-character 'S' (stop) command
over serial to the Arduino running main_robot.ino, matching the protocol
documented in that file's header comment. When the path clears, it does NOT
automatically send 'G' (resume) — see the comment near the publish logic
for why that's a deliberate choice, not an oversight.

ALSO subscribes to CAMERA_SCAN_TOPIC (/camera_scan by default), a second
sensor_msgs/LaserScan expected to come from depthimage_to_laserscan
converting the RealSense D455's depth image (see
obstacle_avoidance_with_camera_launch.py). Same message type, same forward-
cone check, same STOP_DISTANCE_M/FORWARD_CONE_DEG thresholds — reusing one
set of constants for both sources rather than adding camera-specific ones
that have never been tuned against anything. LIDAR and camera are combined
with OR (either source seeing something close enough triggers a stop) and
share the ONE open serial connection below — CLAUDE.md and
wheel_odometry_node.py both flag that opening a second, separate serial
connection to the same port from another node is untested/risky, so this
node stays the single point of contact with the Arduino for obstacle stops.

Bring up the LIDAR driver alone first (as above). The camera path
(depthimage_to_laserscan + RealSense) is separate and untested as of this
writing — the D455 was not physically connected when this was written, so
CAMERA_SCAN_TOPIC's real topic names/remappings need confirming once it is
(see obstacle_avoidance_with_camera_launch.py's header comment).

CONFIGURE BEFORE USE:
  - SERIAL_PORT: the Arduino's port as seen from the SBC/Linux side
                 (check with `ls /dev/ttyACM*` or `ls /dev/ttyUSB*` after
                 plugging in; commonly /dev/ttyACM0)
  - STOP_DISTANCE_M: how close an obstacle must be (in meters) to trigger stop
  - FORWARD_CONE_DEG: how wide a forward-facing angular window to check
  - CAMERA_SCAN_TOPIC: the depth-derived LaserScan topic (see above)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import serial
except ImportError:
    serial = None  # handled at runtime — see _open_serial()


# ---- Tunable parameters — adjust during Week 3 field testing ----
SERIAL_PORT = '/dev/ttyACM0'   # CONFIRM this matches your actual Arduino port
SERIAL_BAUD = 115200           # must match Serial.begin() in main_robot.ino
STOP_DISTANCE_M = 0.40         # stop if anything is closer than 40cm in the forward cone
FORWARD_CONE_DEG = 30          # check +/- 15 degrees around straight-ahead (0 deg)
CAMERA_SCAN_TOPIC = '/camera_scan'  # depth-derived LaserScan, see module docstring


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.serial_conn = self._open_serial()
        self.last_state_blocked = False
        self.lidar_blocked = False
        self.camera_blocked = False

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_scan_callback,
            10
        )
        self.camera_subscription = self.create_subscription(
            LaserScan,
            CAMERA_SCAN_TOPIC,
            self.camera_scan_callback,
            10
        )
        self.get_logger().info(
            f'obstacle_detector started. Stop threshold: {STOP_DISTANCE_M} m, '
            f'forward cone: {FORWARD_CONE_DEG} deg. Sources: /scan (LIDAR), '
            f'{CAMERA_SCAN_TOPIC} (RealSense depth, if publishing).'
        )

    def _open_serial(self):
        if serial is None:
            self.get_logger().error(
                "pyserial not installed. Run: pip install pyserial --break-system-packages "
                "(or apt install python3-serial). Node will run but cannot send stop commands."
            )
            return None
        try:
            conn = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {SERIAL_PORT}')
            return conn
        except serial.SerialException as e:
            self.get_logger().error(
                f'Could not open {SERIAL_PORT}: {e}. '
                f'Check the port name with `ls /dev/ttyACM*` and update SERIAL_PORT.'
            )
            return None

    def lidar_scan_callback(self, msg: LaserScan):
        nearest_in_cone = self._nearest_obstacle_in_forward_cone(msg)
        if nearest_in_cone is not None:
            self.lidar_blocked = nearest_in_cone < STOP_DISTANCE_M
        self._update_combined_state('LIDAR', nearest_in_cone)

    def camera_scan_callback(self, msg: LaserScan):
        nearest_in_cone = self._nearest_obstacle_in_forward_cone(msg)
        if nearest_in_cone is not None:
            self.camera_blocked = nearest_in_cone < STOP_DISTANCE_M
        self._update_combined_state('camera', nearest_in_cone)

    def _update_combined_state(self, source: str, nearest_in_cone):
        # OR, not AND: either sensor alone is enough reason to stop. A scan
        # with no valid readings in the cone (nearest_in_cone is None, e.g.
        # one source hasn't published yet) leaves that source's blocked flag
        # unchanged rather than clearing it — a missing reading is not the
        # same as "confirmed clear".
        is_blocked = self.lidar_blocked or self.camera_blocked

        # Only send a command on a STATE CHANGE, not every scan — sending 'S'
        # repeatedly is harmless (main_robot.ino just re-stops), but flooding
        # serial with unnecessary writes isn't good practice.
        if is_blocked and not self.last_state_blocked:
            detail = f'{nearest_in_cone:.2f} m' if nearest_in_cone is not None else 'prior reading'
            self.get_logger().warn(
                f'Obstacle detected via {source} ({detail}) — sending STOP'
            )
            self._send_command('S')
        elif not is_blocked and self.last_state_blocked:
            self.get_logger().info(
                'Path clear on both sources. '
                'NOT auto-sending resume — see node docstring for why.'
            )
            # Deliberately NOT calling self._send_command('G') here.
            # Auto-resuming the instant a single scan looks clear risks
            # flickering on noisy/borderline readings, and more importantly,
            # resuming motion is a decision with real safety weight that
            # this v1 keeps a human (or a more deliberate state machine,
            # added later) in the loop for. Revisit this once Layer 2 is
            # validated and the team agrees on auto-resume behavior.

        self.last_state_blocked = is_blocked

    def _nearest_obstacle_in_forward_cone(self, msg: LaserScan):
        half_cone_rad = math.radians(FORWARD_CONE_DEG / 2.0)
        nearest = None

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            # Normalize angle to [-pi, pi] so the forward cone (around 0) is
            # checked correctly regardless of how the driver reports angle_min.
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) > half_cone_rad:
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue  # invalid/out-of-spec reading, skip
            if math.isinf(distance) or math.isnan(distance):
                continue

            if nearest is None or distance < nearest:
                nearest = distance

        return nearest

    def _send_command(self, char: str):
        if self.serial_conn is None:
            self.get_logger().warn(f'Cannot send "{char}" — no serial connection.')
            return
        try:
            self.serial_conn.write(char.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
