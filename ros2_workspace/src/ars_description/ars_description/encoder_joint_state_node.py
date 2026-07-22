#!/usr/bin/env python3
"""
encoder_joint_state_node.py
------------------------------------------------------------------------------
Reads the Arduino's "E,<fl>,<fr>,<rl>,<rr>,<millis>" encoder telemetry line
(emitted by main_robot.ino, see its header) and publishes a
sensor_msgs/JointState for the 4 wheel joints in ars_robot.urdf.xacro. With
robot_state_publisher running on that URDF, turning a wheel by hand makes the
wheel spin in RViz in real time (and its TF axes rotate) -- the same live
"model mirrors the real wheels" view this was built for.

This is display-only. It does NOT command the robot and does NOT compute
odometry (that's wheel_odometry_node.py's job). It only turns encoder counts
into wheel angles for visualization.

SHARES the serial-reading approach with wheel_odometry_node.py, but they open
the SAME Arduino port -- run only ONE of them at a time (see that node's
docstring). Both read the same "E," telemetry.

COUNTS -> ANGLE:
  70 counts per wheel revolution (front-left hand-rotation calibration,
  CLAUDE.md 2026-07-17 -- the same figure COUNTS_PER_CM=2.297 was derived
  from), so angle = counts * 2*pi / 70. If a wheel visually spins at the
  wrong speed vs the real one, this constant is the thing to retune.

PER-SIDE SIGN (see CLAUDE.md): right wheels (FR, RR) count ++ on forward
rotation, left wheels (FL, RL) count -- forward, because the motors are
mounted mirrored. The left side is negated below so that hand-rolling ANY
wheel "forward" (top moving toward the robot's front) spins it forward in
RViz too, instead of the left wheels appearing to spin backwards.

CONFIGURE (ROS 2 params, override with -p or in the launch file):
  - port  (default /dev/ttyUSB1): the Arduino. NOTE this is a CH340, so it
    enumerates as ttyUSB*, and the number SHIFTS across reconnects (right
    now the LIDAR is ttyUSB0 and the Arduino ttyUSB1, but that can swap).
    Install the udev rule in ars_description (99-ars-arduino.rules) for a
    stable /dev/arduino symlink and set port:=/dev/arduino instead.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError:
    serial = None  # handled at runtime -- see _open_serial()


COUNTS_PER_REV = 70.0
RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_REV

JOINT_NAMES = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
]


class EncoderJointStateNode(Node):
    def __init__(self):
        super().__init__('encoder_joint_state')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.serial_conn = self._open_serial()
        self.buffer = ''

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        # Poll faster than the Arduino's ~20 Hz (50 ms) telemetry so lines
        # don't pile up in the OS serial buffer.
        self.create_timer(0.01, self._poll_serial)

        self.get_logger().info(
            f'encoder_joint_state started on {self.port} @ {self.baud}. '
            f'{COUNTS_PER_REV} counts/rev. Turn a wheel by hand to see it move in RViz.'
        )

    def _open_serial(self):
        if serial is None:
            self.get_logger().error(
                'pyserial not installed. Run: pip install pyserial --break-system-packages'
            )
            return None
        try:
            conn = serial.Serial(self.port, self.baud, timeout=0)
            self.get_logger().info(f'Connected to Arduino on {self.port}')
            return conn
        except serial.SerialException as e:
            self.get_logger().error(
                f'Could not open {self.port}: {e}. '
                f'Check `ls /dev/ttyUSB*` and set the `port` param (LIDAR and '
                f'Arduino can swap ttyUSB numbers -- see docstring).'
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

    def _handle_line(self, line):
        if not line.startswith('E,'):
            return  # status text from the Arduino, ignore here
        parts = line.split(',')
        if len(parts) != 6:
            return  # partial/malformed line (can happen mid-boot) -- skip
        try:
            fl, fr, rl, rr = (int(parts[1]), int(parts[2]),
                              int(parts[3]), int(parts[4]))
        except ValueError:
            return

        # Per-side sign correction (see docstring): negate left so forward is
        # forward for every wheel.
        positions = [
            -fl * RAD_PER_COUNT,   # front_left
            fr * RAD_PER_COUNT,    # front_right
            -rl * RAD_PER_COUNT,   # rear_left
            rr * RAD_PER_COUNT,    # rear_right
        ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
