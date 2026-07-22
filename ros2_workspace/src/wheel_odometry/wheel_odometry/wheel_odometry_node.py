#!/usr/bin/env python3
"""
wheel_odometry_node.py
------------------------------------------------------------------------------
Reads the "E,<fl>,<fr>,<rl>,<rr>,<millis>" encoder telemetry line added to
arduino/main_robot/main_robot.ino (2026-07-22), converts consecutive count
deltas into a body-frame velocity via mecanum forward kinematics, integrates
a pose, and publishes nav_msgs/Odometry on /odom plus the odom->base_link TF.

Built to replace the frozen identity odom->base_link transform used during
the odometry-free slam_toolbox test (see CLAUDE.md's Current Status) --
slam_toolbox gates whether to add a new scan on real odom movement, so a
transform that never changes made it stop mapping after the first scan.
This node gives it a genuine (if unvalidated) motion source instead.

This node ALSO relays w/a/s/d/q/e/x teleop keys typed into its own terminal
(stdin) straight to the Arduino over the same serial connection it already
has open for reading telemetry -- so driving the robot around for this test
and computing odometry both happen through one process/one serial handle,
rather than needing a second program (e.g. Arduino IDE's Serial Monitor) to
fight over the same port. Type a letter + Enter in the terminal running
this node to drive.

NOTE -- serial port ownership: obstacle_detector_node.py separately writes
'S'/'G' to the same port for the (unrelated) obstacle-stop feature. Running
that node at the same time as this one has not been tested -- run only one
at a time until that's validated.

CONFIGURE BEFORE USE:
  - SERIAL_PORT: the Arduino's port (check `ls /dev/ttyUSB*` -- this is a
    CH340 chip, so it shows up as ttyUSB*, not ttyACM0. No fixed udev symlink
    exists for it yet the way ydlidar has one, so the port number can shift
    across reconnects/re-enumerations -- verify before every run.)
  - WHEELBASE_M / TRACK_WIDTH_M: measured 2026-07-22, not yet cross-checked
    against Haikal's CAD.

NOT YET VALIDATED ON HARDWARE. The mecanum kinematics sign convention below
was cross-checked on paper against every confirmed teleop direction in
main_robot.ino (forward/reverse/strafe/rotate all come out with the expected
sign), but that's a desk check, not a real drive test. Before trusting this
for SLAM: drive straight and confirm vx sign/scale, strafe and confirm vy,
rotate in place and confirm omega -- the same way COUNTS_PER_CM itself was
only trusted after a real F100 test, not from the formula alone.
"""

import math
import select
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None  # handled at runtime -- see _open_serial()


# ---- Tunable parameters -- confirm before use, see module docstring ----
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUD = 115200

# Must match arduino/main_robot/main_robot.ino's COUNTS_PER_CM exactly, or
# distance/velocity here will be wrong. Only calibrated against the
# front-left wheel (2026-07-17) -- applying it to all 4 wheels assumes
# they're close enough (same motor/encoder assembly); flag it if real
# testing shows one wheel's odometry drifting from the others.
COUNTS_PER_CM = 2.297

# Chassis geometry, measured 2026-07-22 (Tsaqif).
WHEELBASE_M = 0.142    # front-to-back distance between wheel centers
TRACK_WIDTH_M = 0.195  # left-to-right distance between wheel centers
LX = WHEELBASE_M / 2.0    # half-wheelbase
LY = TRACK_WIDTH_M / 2.0  # half-track-width

# Matches the teleop keys added to main_robot.ino's handleSerialCommands().
TELEOP_KEYS = set('wsadqex')


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        self.serial_conn = self._open_serial()
        self.buffer = ''
        self.last_counts = None  # (fl, fr, rl, rr, millis)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Poll faster than the Arduino's ~20Hz (50ms) telemetry rate so
        # lines don't pile up in the OS serial read buffer.
        self.create_timer(0.01, self._poll_serial)
        self.create_timer(0.05, self._poll_teleop_stdin)

        self.get_logger().info(
            f'wheel_odometry started. lx={LX:.4f}m ly={LY:.4f}m '
            f'COUNTS_PER_CM={COUNTS_PER_CM}. Type w/a/s/d/q/e/x + Enter here to drive.'
        )

    def _open_serial(self):
        if serial is None:
            self.get_logger().error(
                'pyserial not installed. Run: pip install pyserial --break-system-packages'
            )
            return None
        try:
            conn = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
            self.get_logger().info(f'Connected to Arduino on {SERIAL_PORT}')
            return conn
        except serial.SerialException as e:
            self.get_logger().error(
                f'Could not open {SERIAL_PORT}: {e}. '
                f'Check the port with `ls /dev/ttyUSB*` and update SERIAL_PORT.'
            )
            return None

    def _poll_serial(self):
        if self.serial_conn is None:
            return
        try:
            waiting = self.serial_conn.in_waiting
            if waiting:
                self.buffer += self.serial_conn.read(waiting).decode('utf-8', errors='ignore')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read failed: {e}')
            return

        while '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\n', 1)
            self._handle_line(line.strip())

    def _poll_teleop_stdin(self):
        # Non-blocking stdin check -- select() with 0 timeout returns
        # immediately if nothing has been typed, so this never stalls the
        # ROS2 executor.
        if not select.select([sys.stdin], [], [], 0)[0]:
            return
        typed = sys.stdin.readline().strip()
        if not typed:
            return
        key = typed[0]
        if key not in TELEOP_KEYS:
            self.get_logger().warn(f'Ignoring unrecognized key: {typed!r}')
            return
        if self.serial_conn is None:
            self.get_logger().warn(f'Cannot send {key!r} -- no serial connection.')
            return
        try:
            self.serial_conn.write(key.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def _handle_line(self, line):
        if not line.startswith('E,'):
            # Not telemetry -- this is main_robot.ino's own status text (e.g.
            # "forward", "main_robot ready...", "EMERGENCY STOP"). Surface it
            # instead of silently dropping it -- it's the only way to see
            # whether the Arduino is actually receiving/acking teleop keys,
            # or whether it's still mid-reboot (opening the serial port
            # commonly resets a CH340 board, same as the Arduino IDE's
            # Serial Monitor does).
            if line:
                self.get_logger().info(f'[arduino] {line}')
            return
        parts = line.split(',')
        if len(parts) != 6:
            return  # malformed/partial line (can happen mid-boot) -- skip, don't crash
        try:
            fl, fr, rl, rr, millis = (int(parts[1]), int(parts[2]),
                                       int(parts[3]), int(parts[4]), int(parts[5]))
        except ValueError:
            return

        if self.last_counts is not None:
            self._update_odometry(fl, fr, rl, rr, millis)
        self.last_counts = (fl, fr, rl, rr, millis)

    def _update_odometry(self, fl, fr, rl, rr, millis):
        last_fl, last_fr, last_rl, last_rr, last_millis = self.last_counts
        dt = (millis - last_millis) / 1000.0
        if dt <= 0:
            return  # millis() wrapped or duplicate timestamp -- skip this step

        # Per-side sign correction (see CLAUDE.md): right-side wheels (FR, RR)
        # count ++ on forward rotation, left-side wheels (FL, RL) count --
        # forward -- a normal result of mecanum motors being mounted mirrored
        # on each side, not a wiring mistake. Flip the left side so "positive
        # delta = forward" holds for all 4 wheels, matching setMotor()'s
        # positive-speed-is-forward convention that the kinematics below
        # assumes.
        d_fl = -(fl - last_fl)
        d_fr = (fr - last_fr)
        d_rl = -(rl - last_rl)
        d_rr = (rr - last_rr)

        # Counts -> wheel-surface distance (m) -> velocity (m/s).
        to_m = 1.0 / COUNTS_PER_CM / 100.0
        v_fl = d_fl * to_m / dt
        v_fr = d_fr * to_m / dt
        v_rl = d_rl * to_m / dt
        v_rr = d_rr * to_m / dt

        # Standard mecanum forward kinematics (X-configuration rollers).
        # See module docstring -- desk-checked against every teleop
        # direction, not yet hardware-validated.
        vx = (v_fl + v_fr + v_rl + v_rr) / 4.0
        vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (LX + LY))

        # Integrate body-frame velocity into a world-frame (odom-frame) pose.
        self.theta += omega * dt
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt

        stamp = self.get_clock().now().to_msg()
        self._publish_odometry(vx, vy, omega, stamp)
        self._broadcast_tf(stamp)

    def _publish_odometry(self, vx, vy, omega, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = _yaw_to_quaternion(self.theta)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

    def _broadcast_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = _yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(t)


def _yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy's own SIGINT handler can already call shutdown() before this
        # runs (version-dependent) -- calling it twice raises RCLError on
        # exit. Harmless (happens after the node's real work is done), but
        # noisy -- swallow just this one known-benign double-shutdown case.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
