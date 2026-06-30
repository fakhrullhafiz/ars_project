#!/usr/bin/env python3
"""
obstacle_detector_node.py
------------------------------------------------------------------------------
Layer 2 stretch goal node.

Subscribes to /scan (sensor_msgs/LaserScan), published by the standard
rplidar_ros driver package. Does NOT talk to the RPLIDAR's serial port
directly — that's already handled by rplidar_ros, which is a separate,
well-tested package you should install rather than reimplement:

    sudo apt install ros-jazzy-rplidar-ros

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

CONFIGURE BEFORE USE:
  - SERIAL_PORT: the Arduino's port as seen from the SBC/Linux side
                 (check with `ls /dev/ttyACM*` or `ls /dev/ttyUSB*` after
                 plugging in; commonly /dev/ttyACM0)
  - STOP_DISTANCE_M: how close an obstacle must be (in meters) to trigger stop
  - FORWARD_CONE_DEG: how wide a forward-facing angular window to check
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


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.serial_conn = self._open_serial()
        self.last_state_blocked = False

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info(
            f'obstacle_detector started. Stop threshold: {STOP_DISTANCE_M} m, '
            f'forward cone: {FORWARD_CONE_DEG} deg.'
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

    def scan_callback(self, msg: LaserScan):
        nearest_in_cone = self._nearest_obstacle_in_forward_cone(msg)

        if nearest_in_cone is None:
            return  # no valid readings in the cone this scan, skip

        is_blocked = nearest_in_cone < STOP_DISTANCE_M

        # Only send a command on a STATE CHANGE, not every scan — sending 'S'
        # repeatedly is harmless (main_robot.ino just re-stops), but flooding
        # serial with unnecessary writes isn't good practice.
        if is_blocked and not self.last_state_blocked:
            self.get_logger().warn(
                f'Obstacle detected at {nearest_in_cone:.2f} m — sending STOP'
            )
            self._send_command('S')
        elif not is_blocked and self.last_state_blocked:
            self.get_logger().info(
                f'Path clear (nearest: {nearest_in_cone:.2f} m). '
                f'NOT auto-sending resume — see node docstring for why.'
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
