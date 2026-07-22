#!/usr/bin/env python3
"""
teleop_bridge_node.py
------------------------------------------------------------------------------
Combines encoder_joint_state_node's telemetry reading with keyboard teleop AND
directional LIDAR obstacle avoidance, so ONE process owns the Arduino's serial
port for all of it. Needed because encoder_joint_state_node's own docstring is
explicit: only one process may have the serial port open at a time. Running a
second, separate teleop or obstacle-avoidance script alongside it (e.g. the
obstacle_avoidance package's obstacle_detector_node, which opens its OWN serial
connection to send 'S') would be exactly the untested/risky dual-open this
avoids -- so the safety gating lives HERE, in the one serial owner, rather than
in a second node.

Run this INSTEAD OF encoder_joint_state_node when you want to drive the robot.
For the lightest, most reliable obstacle-avoidance setup, you only need the
LIDAR + this node (no camera, no SLAM):

    Terminal 1: ros2 launch ydlidar_ros2_driver ydlidar_launch.py
    Terminal 2: ros2 run ars_description teleop_bridge --ros-args -p port:=/dev/arduino

It ALSO works unchanged inside robot_slam_camera_teleop_launch.py (that launch
just omits encoder_joint_state_node and expects you to run this by hand) -- the
/scan it subscribes to is published there too, so the safety gating is active
in the full stack as well.

WHY ITS OWN TERMINAL, NOT BUNDLED INTO A LAUNCH FILE: keyboard teleop needs
raw, unbuffered access to this terminal's stdin (so a keypress acts
immediately, no Enter needed) via termios/tty. `ros2 launch` multiplexes
several processes into one terminal and does not give each child its own real,
exclusive TTY for input -- so this must be a plain `ros2 run` in its own
terminal.

KEYS (forwarded as single raw bytes -- see main_robot.ino's
handleSerialCommands(), which expects exactly this, no newline):
    w/s         forward / reverse
    a/d         strafe left / strafe right
    q/e         rotate left / rotate right
    x           stop (clears MANUAL_DRIVE, does not touch EMERGENCY_STOPPED)
    space       EMERGENCY STOP ('S' on the wire)
    g           clear emergency stop ('G' on the wire) -- does NOT auto-resume
    Ctrl-C      quit (restores the terminal to normal mode first)

DIRECTIONAL OBSTACLE AVOIDANCE (added 2026-07-23):
  Subscribes to /scan (LIDAR) and splits it into four 90-degree sectors around
  the robot -- FRONT, BACK, LEFT, RIGHT (0 rad = robot's forward, since the
  URDF mounts laser_frame aligned with base_link). If the nearest valid return
  in a sector is closer than `stop_distance_m` (default 0.30 m), that DIRECTION
  is "blocked". Then:
    - A drive key that would translate INTO a blocked direction is refused
      (w=front, s=back, a=left, d=right) and a warning is logged. The other
      directions still work -- an obstacle in front blocks 'w' only, not
      s/a/d/q/e. This is the whole point: stop moving toward the obstacle,
      keep every other escape route open.
    - Rotation (q/e) and stop (x/space/g) are NEVER gated -- rotating in place
      is not translation toward an obstacle, and you must always be able to
      stop. (Caveat: on a robot with a large footprint, rotating very close to
      a wall could still clip a corner -- this simple sector check does not
      model the chassis shape. It's a movement-direction guard, not a full
      collision-free planner.)
    - If the robot is ALREADY driving in a direction that then becomes blocked
      (obstacle appears, or you drive up to one), an 'x' stop is sent
      automatically. This makes it a real safety stop, not just input
      filtering, because main_robot.ino's teleop drives continuously until the
      next command.
  WHY IN teleop_bridge AND NOT obstacle_detector_node: see the module header --
  single serial owner. WHY 0.30 m: user requirement; tune via the param below.
  Set safety_enabled:=false to drive with the guard off (e.g. to deliberately
  nudge up to something). LIMITATIONS worth knowing before trusting it: the
  LIDAR only sees its own scan plane (obstacles below/above that height are
  invisible), and it cannot see closer than its ~0.10 m range_min -- an object
  jammed right against the robot may read as an invalid 0.0 and be ignored.

Telemetry parsing (COUNTS_PER_REV, per-side sign correction, /joint_states
publishing) is copied from encoder_joint_state_node.py rather than imported --
see CLAUDE.md's Working Conventions on why the Arduino sketches duplicate their
constants instead of sharing a header: small duplicated blocks are preferred
over a shared-module abstraction while the team is still learning the codebase.
If COUNTS_PER_REV or the sign convention ever changes, update both node files.
"""
import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan

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
# to the wire protocol's 'S'/'G' emergency-stop commands.
KEY_TO_WIRE = {
    'w': 'w', 's': 's', 'a': 'a', 'd': 'd', 'q': 'q', 'e': 'e', 'x': 'x',
    ' ': 'S',
    'g': 'G',
}

# Which translation direction each drive key moves the robot toward, for the
# obstacle guard. Keys not listed here (q/e rotate, x/space/g stop) are never
# gated -- see the docstring.
KEY_TO_DIRECTION = {
    'w': 'front',
    's': 'back',
    'a': 'left',
    'd': 'right',
}

DIRECTIONS = ('front', 'back', 'left', 'right')


class TeleopBridgeNode(Node):
    def __init__(self):
        super().__init__('teleop_bridge')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        # Obstacle-guard params (see docstring). safety_enabled lets you turn
        # the whole guard off; stop_distance_m is the 30 cm requirement;
        # sector_half_angle_deg must stay < 90 so the four sectors don't
        # overlap (45 => four clean 90-degree quadrants tiling the full circle).
        self.declare_parameter('safety_enabled', True)
        self.declare_parameter('stop_distance_m', 0.30)
        self.declare_parameter('sector_half_angle_deg', 45.0)
        # Debug aid: when true, prints the nearest distance per sector ~1/s so
        # you can SEE what the guard sees (is /scan arriving? is the object in
        # the sector you think?). Off by default so normal runs stay quiet.
        self.declare_parameter('log_sectors', False)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.safety_enabled = bool(self.get_parameter('safety_enabled').value)
        self.stop_distance = float(self.get_parameter('stop_distance_m').value)
        self.sector_half_angle = math.radians(
            float(self.get_parameter('sector_half_angle_deg').value))
        self.log_sectors = bool(self.get_parameter('log_sectors').value)

        self.serial_conn = self._open_serial()
        self.buffer = ''

        # Obstacle-guard state. blocked[dir] is True while a sector has an
        # obstacle within stop_distance. active_direction is the translation
        # direction the robot is currently commanded to move (None for
        # rotate/stop), used to auto-stop if that direction becomes blocked.
        self.blocked = {d: False for d in DIRECTIONS}
        self.active_direction = None

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        # LIDAR publishes /scan as Best Effort sensor data -- a default
        # (Reliable) subscription silently receives NOTHING (documented QoS
        # gotcha, see CLAUDE.md), which would leave the guard permanently
        # "clear" and unsafe. qos_profile_sensor_data matches it.
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, qos_profile_sensor_data)

        # Poll faster than the Arduino's ~20 Hz telemetry so lines don't pile up.
        self.create_timer(0.01, self._poll_serial)
        # Poll keyboard at 50 Hz -- responsive, and cheap (non-blocking select).
        self.create_timer(0.02, self._poll_keyboard)

        self._old_term_settings = None
        self._enter_raw_terminal_mode()

        guard = (f'ON ({self.stop_distance:.2f} m)' if self.safety_enabled
                 else 'OFF')
        self.get_logger().info(
            f'teleop_bridge started on {self.port} @ {self.baud}. '
            f'Obstacle guard: {guard}. '
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
                f'Arduino can swap ttyUSB numbers -- prefer port:=/dev/arduino).'
            )
            return None

    # ---- obstacle guard --------------------------------------------------

    def _sector_of(self, angle):
        """Classify a scan angle (rad) into 'front'/'back'/'left'/'right'.

        0 rad is the robot's forward (laser_frame is mounted aligned with
        base_link, see the URDF). Angles increase CCW (REP-103): +pi/2 is left,
        -pi/2 is right, +/-pi is behind. With sector_half_angle < pi/2 the four
        90-degree quadrants tile the circle with no gaps or overlaps.
        """
        a = math.atan2(math.sin(angle), math.cos(angle))  # normalize to [-pi, pi]
        abs_a = abs(a)
        if abs_a <= self.sector_half_angle:
            return 'front'
        if abs_a >= math.pi - self.sector_half_angle:
            return 'back'
        return 'left' if a > 0.0 else 'right'

    def _scan_callback(self, msg: LaserScan):
        if not self.safety_enabled:
            return

        mins = {d: math.inf for d in DIRECTIONS}
        angle = msg.angle_min
        for r in msg.ranges:
            # Skip invalid returns: 0.0/NaN/inf and anything outside the
            # sensor's own valid window. A jammed-too-close obstacle can read
            # as 0.0 here and be missed -- inherent to the LIDAR's range_min.
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                sector = self._sector_of(angle)
                if r < mins[sector]:
                    mins[sector] = r
            angle += msg.angle_increment

        for d in DIRECTIONS:
            self.blocked[d] = mins[d] < self.stop_distance

        if self.log_sectors:
            def fmt(x):
                return f'{x:.2f}' if math.isfinite(x) else '  -- '
            blocked_now = [d for d in DIRECTIONS if self.blocked[d]] or ['none']
            # throttle so the teleop terminal isn't flooded (keys still work).
            self.get_logger().info(
                f'sectors[m] F={fmt(mins["front"])} B={fmt(mins["back"])} '
                f'L={fmt(mins["left"])} R={fmt(mins["right"])}  '
                f'blocked={",".join(blocked_now)}',
                throttle_duration_sec=1.0,
            )

        # Real safety stop: if we're actively driving into a direction that is
        # now blocked, halt -- teleop drives continuously until the next
        # command, so filtering future keypresses alone wouldn't stop motion
        # already underway.
        if (self.active_direction is not None
                and self.blocked[self.active_direction]
                and self.serial_conn is not None):
            self.serial_conn.write(b'x')
            self.get_logger().warn(
                f'Obstacle within {self.stop_distance:.2f} m to '
                f'{self.active_direction} while moving -- STOPPING.'
            )
            self.active_direction = None

    # ---- keyboard --------------------------------------------------------

    def _enter_raw_terminal_mode(self):
        if not sys.stdin.isatty():
            self.get_logger().warn(
                'stdin is not a real terminal -- keyboard teleop will not work '
                '(run this with `ros2 run`, not piped/redirected).'
            )
            return
        self._old_term_settings = termios.tcgetattr(sys.stdin)
        # cbreak (not raw): single keypresses arrive immediately without Enter,
        # but Ctrl-C still raises KeyboardInterrupt so quitting is normal.
        tty.setcbreak(sys.stdin.fileno())

    def _restore_terminal_mode(self):
        if self._old_term_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term_settings)

    def _poll_keyboard(self):
        if not sys.stdin.isatty() or self.serial_conn is None:
            return
        # Drain every key currently waiting so a burst doesn't lag behind.
        while select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()
            wire_cmd = KEY_TO_WIRE.get(key)
            if wire_cmd is None:
                continue

            direction = KEY_TO_DIRECTION.get(key)  # None for rotate/stop keys
            # Refuse to drive INTO a blocked direction; everything else passes.
            if (self.safety_enabled and direction is not None
                    and self.blocked[direction]):
                self.get_logger().warn(
                    f"BLOCKED: obstacle within {self.stop_distance:.2f} m to "
                    f"{direction} -- ignoring '{key}'. Other directions still work."
                )
                continue

            self.serial_conn.write(wire_cmd.encode())
            # Track what we're now doing so _scan_callback can auto-stop if this
            # direction becomes blocked. Rotate/stop/e-stop clear it (None).
            self.active_direction = direction

    # ---- Arduino telemetry -> /joint_states ------------------------------

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
            return  # status text from the Arduino (e.g. "forward") -- ignore here
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
