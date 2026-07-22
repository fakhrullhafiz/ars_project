#!/usr/bin/env python3
"""
teleop_bridge_node.py
------------------------------------------------------------------------------
Combines encoder_joint_state_node's telemetry reading with keyboard teleop,
so ONE process owns the Arduino's serial port for both jobs at once. Needed
because encoder_joint_state_node's own docstring is explicit: only one
process may have the serial port open at a time. Running a second, separate
teleop script alongside encoder_joint_state_node (or wheel_odometry_node) is
exactly the untested/risky dual-open this avoids.

Run this INSTEAD OF encoder_joint_state_node when you want to actually drive
the robot while everything else (LIDAR, Cartographer, RealSense, RViz) is up
-- see robot_slam_camera_teleop_launch.py, which launches everything except
the wheel-telemetry node, expecting you to run this one by hand in its own
terminal.

WHY ITS OWN TERMINAL, NOT BUNDLED INTO THE LAUNCH FILE: keyboard teleop needs
raw, unbuffered access to this terminal's stdin (so a keypress acts
immediately, no Enter needed) via termios/tty. `ros2 launch` multiplexes
multiple nodes' output into one terminal and does not give each child
process its own real, exclusive TTY for input -- so this must be run as a
plain `ros2 run` in a terminal of its own, not as a Node() in a launch file.

KEYS (forwarded as single raw bytes -- see main_robot.ino's
handleSerialCommands(), which expects exactly this, no newline):
    w/s         forward / reverse
    a/d         strafe left / strafe right
    q/e         rotate left / rotate right
    x           stop (clears MANUAL_DRIVE, does not touch EMERGENCY_STOPPED)
    space       EMERGENCY STOP ('S' on the wire) -- separate from 'x' because
                main_robot.ino gates teleop off entirely while
                EMERGENCY_STOPPED; you need a way in to trigger it and 'G' to
                clear it without leaving raw keyboard mode.
    g           clear emergency stop ('G' on the wire) -- does NOT auto-resume
                driving, matching main_robot.ino's deliberate design.
    Ctrl-C      quit this node (restores the terminal to normal mode first)

Telemetry parsing (COUNTS_PER_REV, per-side sign correction, /joint_states
publishing) is copied from encoder_joint_state_node.py rather than imported
-- see CLAUDE.md's Working Conventions on why motor_control.ino/main_robot.ino
duplicate their pin/motor constants instead of sharing a header: this
project deliberately keeps small duplicated blocks over a shared-module
abstraction while the team is still learning the codebase. If
COUNTS_PER_REV or the sign convention ever changes, update both node files.
"""
import math
import select
import sys
import termios
import tty

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

# Keyboard key -> byte written to the Arduino. 'x'/'w'/'s'/'a'/'d'/'q'/'e' map
# straight through (main_robot.ino reads them directly); space/g are remapped
# to the wire protocol's 'S'/'G' since those are the actual emergency-stop
# commands, not the teleop-direction commands.
KEY_TO_WIRE = {
    'w': 'w', 's': 's', 'a': 'a', 'd': 'd', 'q': 'q', 'e': 'e', 'x': 'x',
    ' ': 'S',
    'g': 'G',
}


class TeleopBridgeNode(Node):
    def __init__(self):
        super().__init__('teleop_bridge')

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
        # Poll keyboard input at 50 Hz -- plenty responsive for teleop, and
        # cheap since it's just a non-blocking select() check.
        self.create_timer(0.02, self._poll_keyboard)

        self._old_term_settings = None
        self._enter_raw_terminal_mode()

        self.get_logger().info(
            f'teleop_bridge started on {self.port} @ {self.baud}. '
            f'Keys: w/a/s/d/q/e drive, x stop, SPACE emergency-stop, '
            f'g clear-emergency-stop, Ctrl-C to quit.'
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

    def _enter_raw_terminal_mode(self):
        if not sys.stdin.isatty():
            self.get_logger().warn(
                'stdin is not a real terminal -- keyboard teleop will not work '
                '(run this with `ros2 run`, not piped/redirected).'
            )
            return
        self._old_term_settings = termios.tcgetattr(sys.stdin)
        # cbreak (not raw): single keypresses arrive immediately without
        # needing Enter, but Ctrl-C still raises KeyboardInterrupt normally
        # so quitting doesn't require anything special.
        tty.setcbreak(sys.stdin.fileno())

    def _restore_terminal_mode(self):
        if self._old_term_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term_settings)

    def _poll_keyboard(self):
        if not sys.stdin.isatty() or self.serial_conn is None:
            return
        # Drain every key currently waiting, not just one, so a burst of
        # fast keypresses doesn't lag behind.
        while select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            wire_cmd = KEY_TO_WIRE.get(key.lower())
            if wire_cmd is not None:
                self.serial_conn.write(wire_cmd.encode())

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
            return  # status text from the Arduino (e.g. "forward", "EMERGENCY STOP") -- ignore here
        parts = line.split(',')
        if len(parts) != 6:
            return  # partial/malformed line (can happen mid-boot) -- skip
        try:
            fl, fr, rl, rr = (int(parts[1]), int(parts[2]),
                              int(parts[3]), int(parts[4]))
        except ValueError:
            return

        # Per-side sign correction (see CLAUDE.md): negate left so forward is
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

    def destroy_node(self):
        self._restore_terminal_mode()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridgeNode()
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
