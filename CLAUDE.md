# CLAUDE.md

Context for Claude Code when working in this repo. Read this before making changes — especially anything touching motor control or power.

## What this project is

A 3-person, 4-week university robotics project: a mecanum-wheel autonomous mobile robot (AMR). The graded bar is low (move from Point A to Point B), so this repo deliberately prioritizes a working baseline over feature completeness. Don't add complexity that isn't asked for in a given task — see "Priority layers" below before suggesting scope expansions.

Full hardware specs live in `Documentation/robotics_component_documentation.md` in the shared Google Drive (not in this repo — this repo is code-only). Ask the user for the relevant section if you need exact pin numbers, voltage specs, or part numbers rather than guessing.

## Team & scope

- **Pekol (fakhrullhafiz) — Person B:** Arduino / motion control. This is the primary scope for work in this repo's `arduino/` folder.
- **Haikal — Person A:** mechanical & power systems (no code folder — CAD/wiring lives in Drive).
- **Tsaqif — Person C:** ROS2 / perception (YDLidar, RealSense, ROS2 Jazzy) — `ros2_workspace/`.

(`README.md`'s team table currently lists these roles without names — worth syncing at some point, but not done here since it wasn't asked for.)

## Priority layers (do not skip ahead)

1. **Layer 0 — Baseline:** Arduino Mega drives all 4 motors via the MC33886VW drivers, robot physically moves forward/turns. This alone satisfies the assignment.
2. **Layer 1 — Reliable navigation:** encoder feedback (JGB37-520 magnetic encoders) used for closed-loop "drive to distance" / "turn to angle", so A→B is repeatable, not timing-based.
3. **Layer 2 — Stretch:** YDLidar-based obstacle detection layered on top of Layer 1.
4. **Layer 3 — Optional stretch:** Axiomtek SBC + RealSense D455 + ROS2 Nav2/SLAM. Only relevant once Layers 0–2 are solid and rehearsed.

If asked to work on something from a higher layer, check whether the layer below it is confirmed working first. If unclear, ask rather than assuming it's done. Never let stretch-goal work (Layer 2/3) block or delay a required lower layer.

## Critical hardware facts — do not regenerate these from training data, treat as ground truth

- **Motors (JGB37-520):** rated 6.0V DC. Integrated magnetic encoder on a 6-pin JST connector: red = motor+, white = motor−, blue = encoder VCC, black = encoder GND, yellow = encoder phase A, green = encoder phase B.
- **Battery: 2S LiPo, 7.4V nominal / 8.4V full charge, XT60 connector. Confirmed** — supersedes the earlier "3S or 4S, TBD" placeholder that was here. If any code, comment, or teammate reference still assumes 3S/4S, that's now stale (see the MAX_PWM conflict noted below).
- **Battery power distribution (lever-nut block, 2026-07-02):** the XT60 battery lead feeds a high-current **2-in / 4-out lever-nut distribution block** — this is the sanctioned fan-out for battery power and **replaces the earlier hand-made-splitter idea; no breadboard is anywhere on the power path.** The block has two internally-isolated bus bars: **blue levers = battery positive rail, orange levers = battery negative rail.** Two parallel output channels:
  - **Channel 1** → driver 1 VCC/GND — front motors (FL + FR).
  - **Channel 2** → driver 2 VCC/GND — rear motors (RL + RR).
  - The **SZDULI 19V converter** (SBC power) splices into one of the block's high-current output lines — electrically a parallel tap on the same battery rail, *not* downstream of a driver.
  Implications for hardware-init code, pinning, and troubleshooting: (a) the block only *distributes*, it does not regulate — every output sits at raw battery voltage, so the `MAX_PWM` voltage-mismatch reasoning below is unchanged. (b) **The Arduino Mega is NOT powered from this block** — it runs off USB; the driver *logic* headers (5V+/GND) are fed from the Mega's 5V/GND, a **separate power domain** from the driver VCC/GND *motor-power* terminals fed here — keep the two straight when diagnosing. (c) **Common ground:** the Mega GND must still tie to the orange (battery-negative) rail so motor-control signals share a reference with the drivers. (d) Both drivers *and* the SBC converter now draw through the single XT60 input — confirm the input lever's current rating covers the combined load, and note that motor stall current can momentarily sag the shared rail (an `S` emergency stop, which zeroes PWM, also relieves it).
- **Driver (MC33886VW dual H-bridge, x2 modules):** 6-pin logic header per module (GND, IN1–IN4, 5V+) plus 3 screw-terminal pairs (OUT1/2, VCC/GND, OUT3/4). No separate PWM/enable pin — speed is set via `analogWrite()` directly on the IN pins (see `setMotor()` in `motor_control.ino`).
- **This is a voltage mismatch by design, not a bug.** The MC33886VW driver passes battery voltage straight through to the motors — it does not regulate it down. The fix is software: cap PWM duty cycle rather than running motors at full duty cycle. **With the confirmed 2S battery (8.4V max), the correct ceiling is `MAX_PWM = 180` out of 255 (~70%, since 6.0V / 8.4V ≈ 71%)** — tune down further only if real motor-temperature testing says so. **Never write or suggest motor control code that defaults to 100% PWM duty cycle** without an explicit, deliberate reason — this risks motor damage given the voltage mismatch.
  - `arduino/motor_control/motor_control.ino` and `arduino/main_robot/main_robot.ino` both set `MAX_PWM = 180` (2026-07-01, confirmed with user) — **not yet validated on hardware.** Treat 180 as the theoretical ceiling for the confirmed 2S pack, not a proven-safe running value, until someone has actually run the motors and checked casing temperature after a multi-minute test.
- **SZDULI Y2-K101908 converter:** steps battery voltage to a clean 19V rail for the Axiomtek SBC, rated up to 152W. Draws its input from one of the distribution block's output lines (see the power-distribution bullet above). Not a bottleneck — don't spend effort here unless explicitly asked.
- **Axiomtek SBC confirmed specs (2026-07-20):** Intel Core i7-1185G7E (11th-gen Tiger Lake, 4C/8T, 2.8GHz base), x86_64 — resolves the CPU-variant open question below; a real embedded-class i7, not a low-end Celeron, and matches the dev VM's architecture so prebuilt ROS2 Jazzy packages install the same way. OS is Ubuntu 24.04.4 LTS with `ros-jazzy-desktop` installed via apt. Two software gaps found on first check (not hardware, status as of 2026-07-20 unconfirmed whether fixed): `ROS_DISTRO` isn't sourced in a fresh shell (needs `source /opt/ros/jazzy/setup.bash`, probably belongs in `.bashrc`), and `ros2 --version`/`ros2 doctor` failed with a `ros2cli` package-metadata error (`PackageNotFoundError` via `importlib.metadata`) on first check — a real bug to fix, not just an unsourced-environment issue.
- **LIDAR is a YDLidar, NOT an RPLIDAR A1 — corrects earlier assumption (2026-07-21).** Every prior reference in this repo to RPLIDAR/`rplidar_ros` was wrong; the physical unit is a YDLidar, driven by the vendored `ydlidar_ros2_driver` package (see Architecture below) rather than an apt-installed driver, so the earlier `ros-jazzy-rplidar-ros` apt-install step is no longer applicable. Exact model unconfirmed from hardware alone — `ros2_workspace/src/ydlidar_ros2_driver/params/ydlidar.yaml`'s comments ("TRIANGLE for X2", "X2 is single-channel") suggest an **X2**, but get this confirmed against the physical unit/box before treating it as final. USB-serial adapter shows up as `/dev/ttyUSB0` (CP210x), 115200 baud — matches earlier SBC diagnostics.
- **Vention 4-port USB 3.0 hub (Model CHL) — the planned SBC sensor hub, power adapter NOT included.** Meant to carry the Arduino Mega + RealSense D455 + YDLidar off the SBC; as of 2026-07-20 none of them are wired through it yet — RealSense is plugged into the SBC's own built-in USB2 hub instead (self-powered, 100mA, also carrying keyboard/mouse/Bluetooth), which caps its bandwidth below full-res depth/color. The Vention hub is externally-powered by design (Micro-USB 5V/2A input, up to 900mA/port, USB 3.0) but ships without that adapter — **running it on bus power only risks the exact RealSense/LIDAR voltage-drop disconnects it was bought to prevent.** Don't treat the SBC's sensor wiring as done until (a) the 5V/2A adapter is in hand and plugged into the hub and (b) RealSense/LIDAR/Arduino are actually moved onto it.
- **Arduino Mega 2560:** ATmega2560, 5V logic, 54 digital I/O pins, hardware Serial0–3, I2C on pins 20/21. Motor IN pins on D2–D9 (PWM-capable); only 6 pins support true hardware interrupts (D2, D3, D18, D19, D20, D21) — see the 4-encoder pin plan below for how all 4 encoders are allocated across those plus the Mega's PCINT0 group.

If any of these facts appear to conflict with something in code or in a teammate's comment, flag the discrepancy to the user rather than silently picking one.

## Current status — update as things actually change, don't let this go stale

- **Full rewire completed 2026-07-16:** all 4 motors and all 4 encoders are now physically wired (breadboard used for logic-side 5V/GND fan-out only; battery power to both driver modules is a soldered/twisted splice with heat shrink, not breadboard). This supersedes the earlier "rear pair blocked, needs a splitter" note and the lever-nut distribution block note above — both are superseded by whichever wiring is physically in place now; treat `encoder_test.ino` and this section as ground truth.
  - Front-left motor: D2/D3, driver 1 channel A.
  - Front-right motor: D4/D5, driver 1 channel B.
  - Rear-left motor: D6/D7, driver 2 channel A.
  - Rear-right motor: D8/D9, driver 2 channel B.
  - **All 4 motors' spin direction confirmed correct post-rewire (2026-07-17)**, via `motor_control.ino`'s single-motor ID test (`1`-`8`) and combined forward/reverse/strafe/rotate (`w/s/a/d/q/e`) — every wheel matched its assigned rotation, and strafe/rotate also matched on the ground. No wiring swaps were needed. `MAX_PWM` in `motor_control.ino`, temporarily lowered to 80 for this test, is now restored to 180. Thermal check (see below) is still outstanding.
- **4-encoder pin plan is the real PCINT-based wiring, not the earlier "phase A only" plan.** Only 6 Mega pins (D2, D3, D18, D19, D20, D21) support true hardware interrupts, and D2/D3 are taken by the front-left motor, so only FL and FR get full interrupt-pin pairs; RL and RR use the Mega's PCINT0 group instead (raw AVR registers, not a library — see `encoder_test.ino`). This corrects the earlier "phase A only needs a true interrupt pin" assumption (D18-D21 phase A / D22-D25 phase B), which turned out not to match how the board's interrupt pins actually needed to be allocated. All 4 encoders now wired:
  - Front-left: A=D18 / B=D19 (hardware interrupt).
  - Front-right: A=D20 / B=D21 (hardware interrupt).
  - Rear-left: A=D11 / B=D24 (pin-change interrupt — D11 is not a hardware-interrupt pin).
  - Rear-right: A=D12 / B=D25 (pin-change interrupt).
  - **`main_robot.ino` now tracks all 4 encoders too (2026-07-17)**, ported from `encoder_test.ino`'s proven implementation. Only `flCount` drives the `F<cm>` stop condition, unchanged from before — `frCount`/`rlCount`/`rrCount` are logged for diagnostics only (e.g. spotting a wheel that's under/over-rotating), since `COUNTS_PER_CM` was only ever calibrated against FL. Using the other 3 wheels for the actual stop/steering logic is a distinct future step that would need its own hardware validation first.
- **All 4 encoders confirmed reading correctly via `encoder_test.ino` (2026-07-17).** Rear-right initially read nothing at all — its phase-A (yellow) lead was physically on D10 instead of D12; reseating it on D12 fixed it. If any encoder ever goes silent again, check the physical connector/pin before suspecting the ISR code, since the PCINT0 handler is shared and proven correct by the other 3 wheels working.
  - **Sign convention confirmed and expected, not a bug:** right-side wheels (FR, RR) count `++` on forward rotation / `--` on reverse; left-side wheels (FL, RL) count the opposite (`--` forward / `++` reverse). This is the normal result of mecanum drivetrains mounting left- and right-side motors mirrored, so it's not a wiring mistake to chase — but it does mean any future firmware that turns these counts into steering/distance decisions (e.g. using `main_robot.ino`'s now-tracked `frCount`/`rlCount`/`rrCount` for more than diagnostics) must apply a per-side sign correction so "drive forward" reads positive on all 4 wheels.
- **`COUNTS_PER_CM` calibrated and sanity-checked (2026-07-17):** `2.297`, derived from the front-left wheel's encoder (70 counts per hand-rotated revolution, 97mm wheel diameter) via `encoder_test.ino`, replacing the old placeholder (`20.0`). Confirmed against real hardware via `main_robot.ino`'s `F100` command (100cm commanded, actual stop landed close to the 100cm mark, run at a temporarily-lowered `MAX_PWM=100` to isolate calibration error from coasting overshoot — see `main_robot.ino`). Good enough to trust for now; still based on a single revolution + single distance trial, so a repeat measurement would tighten it further if precision becomes important later.
- **Thermal check at `MAX_PWM=180` passed for all 4 motors + both drivers (2026-07-17).** Ran forward/reverse/strafe/rotate in short bursts on the ground (real load, not free-spinning), touching each motor casing and both driver boards between bursts across several cumulative minutes — everything stayed comfortably warm, no motor ran notably hotter than the others. `180` is now a validated running value for all 4 wheels, not just the pre-rewire front-left result.
- **SBC bring-up, 2026-07-20 to 2026-07-21:** LIDAR mount still not printed (Haikal) — doesn't block bring-up since the LIDAR isn't on the robot chassis yet anyway. **RealSense confirmed working** (visual check passed). **LIDAR confirmed producing a roughly-correct laser-scan visual** — good enough to call Layer 2's sensor side minimally rehearsed, though not yet integrated end-to-end with `obstacle_detector_node.py`'s stop logic. Remaining SBC-side blockers: `ros2cli` package-metadata error found during initial diagnostics (fix status unconfirmed), Vention hub still has no power adapter, and all 3 devices (Arduino/RealSense/LIDAR) are currently on the SBC's built-in ports, not the Vention hub — fine for now, just not the final wiring.
- **Repo sync problem found and partially fixed (2026-07-21):** Tsaqif's Claude session had a long-stale, git-less local copy of this repo (`~/ars_project` on the SBC, un-synced since roughly Week 1) and committed straight from it to a new `person-c-perception` branch, which reverted `main_robot.ino`/`motor_control.ino` to their pre-4-encoder-rewire state, deleted `NEXT_SESSION.md`, and deleted the `obstacle_avoidance` package. **That branch was NOT merged into `main` and `main` was never affected.** The one genuinely new, wanted piece — the vendored `ydlidar_ros2_driver` package — was cherry-picked out onto a clean branch built from current `main` instead (see `perception-ydlidar` branch); `obstacle_avoidance` did not need to be deleted for this, since its code has no RPLIDAR-specific dependency, only now-corrected docstring text. Still to resolve with Tsaqif directly: confirm which SBC directory (`~/ars_project` vs. the fresh `~/ars_project_repo` clone) is actually tracking `origin` going forward, since the stale one is what caused this.

## Repo layout
CAD files, the project plan, the report draft, and assembly photos are in Google Drive, not here. If you need them, ask the user to paste/upload the relevant content rather than assuming they're accessible from this repo.

## Architecture — how the pieces fit together

**`arduino/` — three sketches, a progression, not three independent programs:**
- `encoder_test.ino` — bring-up/calibration only. Reads all 4 encoders (FL/FR via hardware interrupt, RL/RR via pin-change interrupt since only 6 Mega pins support true hardware interrupts) and prints raw pulse counts so a teammate can hand-derive `COUNTS_PER_CM` per wheel. Not meant to run on the finished robot.
- `motor_control.ino` — Layer 0 baseline. Open-loop, keyboard-driven (`w/a/s/d/q/e/x` over serial) mecanum kinematics. `setMotor()`/`stopAll()`/pin assignments here are the canonical reference — `main_robot.ino` duplicates them and must be kept in sync by hand (no shared header exists yet).
- `main_robot.ino` — Layer 1, supersedes `motor_control.ino` for actual runs. Same motor/pin layout, plus all 4 encoders (FL/FR interrupt-driven, RL/RR pin-change) for closed-loop "drive N cm" — though only FL's count drives the stop condition, the other 3 are diagnostics-only for now (see "Current status") — and a text serial command protocol that is the integration point with the ROS2 side.

**Serial protocol (the Arduino ↔ ROS2 boundary):** plain ASCII over USB serial, 115200 baud — `F<distance_cm>` (drive forward and stop), `S` (emergency stop), `G` (clear stop, does not auto-resume). Defined in `main_robot.ino`'s header comment; `obstacle_detector_node.py` is the only current consumer. Treat this protocol as a contract — changing the command characters or format requires updating both sides together.

**`ros2_workspace/` — two packages:**
- `obstacle_avoidance` (Layer 2, `ament_python` build type): `obstacle_detector_node.py` subscribes to `/scan` (a standard `sensor_msgs/LaserScan` topic — the node has no brand-specific dependency on whichever driver publishes it), finds the nearest reading inside a forward-facing angular cone, and writes `'S'` to the Arduino's serial port on the transition from clear → blocked. It deliberately never auto-sends `'G'` — resuming motion is treated as a decision that should require a human or a more deliberate state machine, not a bare distance threshold.
- `ydlidar_ros2_driver` (vendored, not apt-installed — added 2026-07-21): publishes `/scan` from the physical YDLidar unit (see Critical hardware facts). Vendored as plain tracked files rather than a git submodule so `colcon build` works straight out of a normal clone. It's a ROS2 **lifecycle node** — after launching, if `/scan` doesn't appear, it likely needs manual `ros2 lifecycle set /ydlidar_ros2_driver_node configure` then `activate`, since neither vendored launch file auto-transitions it.
- The LIDAR driver and the `obstacle_avoidance` node are brought up as two separate `ros2 launch` steps (see Commands below) so a scan-data problem and a detection-logic problem aren't debugged as one blob.

## Commands

**Arduino (`arduino/`):** no `arduino-cli`/PlatformIO config in this repo — compile and upload through the Arduino IDE (board: Arduino Mega or Mega 2560). There is no automated build here; "compiles in the IDE" is a weaker signal than "ran on hardware" — see Working conventions below.

**ROS2 (`ros2_workspace/`, package `obstacle_avoidance`, distro Jazzy):**
```bash
cd ros2_workspace
colcon build
source install/setup.bash

# bring up the LIDAR driver first, in its own terminal, and confirm real data:
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
# if /scan doesn't show up, the lifecycle node likely needs manual activation:
#   ros2 lifecycle set /ydlidar_ros2_driver_node configure
#   ros2 lifecycle set /ydlidar_ros2_driver_node activate
ros2 topic echo /scan

# then, in another terminal:
ros2 launch obstacle_avoidance obstacle_avoidance_launch.py
# (or: ros2 run obstacle_avoidance obstacle_detector)
```
No tests exist yet — `package.xml`'s `ament_flake8`/`ament_pep257`/`pytest` entries are unused scaffolding from `ros2 pkg create`, not an active test suite.

## Working conventions

- **Test on hardware before suggesting a merge to `main`.** "Compiles successfully" is not the same as "drives the real motors correctly" — say this explicitly when handing code back, don't imply it's done.
- **Small, reviewable diffs.** Avoid large multi-file rewrites in one pass unless asked; this is a team of beginners and large diffs are hard for them to review and learn from.
- **Always explain *why*, briefly, for anything touching motor PWM limits, encoder calibration constants, or safety stop logic** — these are the values most likely to need re-tuning by a teammate who isn't deeply familiar with the code.
- **Don't introduce ROS2 into the Arduino-side code, and don't introduce Arduino-specific assumptions into the ROS2 workspace.** The two communicate over a simple serial interface (see `ars_project/README.md` for current protocol status) — keep that boundary clean.
- **Ask before adding new dependencies/libraries**, especially on the ROS2 side — install footprint and version compatibility (this team uses Ubuntu 24.04, ROS2 Humble or Jazzy — confirm which before assuming) matters for a project where setup time is scarce.
- **A wrong spin direction is a wiring fix, not a firmware fix.** Swap the two OUT leads on that channel at the driver's screw terminals. Don't "solve" reversed rotation by flipping signs in `setMotor()` or the mecanum kinematics functions.
- **Folder ownership is real, not just a README table.** `arduino/` is Pekol's day-to-day, `ros2_workspace/` is Tsaqif's. Cross-folder edits are fine when asked for, but call it out explicitly rather than doing it silently.
- **Commits:** descriptive subject line, plus a short "why" in the body when it isn't obvious from the diff alone.

## Known open questions (update as resolved)

- [x] Battery confirmed: 2S LiPo, 7.4V nominal / 8.4V full charge, XT60 (see Critical hardware facts). Supersedes the earlier 3S/4S placeholder. Pack health still unverified.
- [x] Confirmed Axiomtek SBC CPU variant: Intel Core i7-1185G7E, x86_64 (2026-07-20, see Critical hardware facts). Not a Celeron; architecture matches the dev VM. DMI board/vendor strings read as generic placeholders rather than "Axiomtek" — software alone can't confirm this is the physically-branded unit, worth a chassis-label glance if that ever matters.
- [x] ROS2 distro — Jazzy (this dev machine and the SBC both confirmed `ROS_DISTRO=jazzy` / Ubuntu 24.04). Flag it if a teammate's machine has Humble instead.
- [ ] Confirmed exact YDLidar model — `params/ydlidar.yaml`'s comments suggest X2, but this hasn't been checked against the physical unit/box (see Critical hardware facts). Matters because the vendored driver ships separate per-model param files (X2.yaml, G1.yaml, etc.) with different settings.
- [x] Serial protocol between Arduino and the obstacle-avoidance node — defined in `main_robot.ino`'s header: `F<distance_cm>`, `S`, `G` (see Architecture above).

Keep this list current. A stale "known open questions" list is worse than none — if something here gets resolved, delete it or check it off rather than leaving it stale for the next session to trip over.
