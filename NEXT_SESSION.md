# Next lab session — 2026-07-20

Working plan for the next time the team is together at the robot. Update or
replace this file after the session rather than letting it go stale — it's
meant to reflect the *next* upcoming session, not a running log (see
`README.md`'s Status section for the running log).

For hardware facts, pin plans, and the "why" behind PWM/calibration values,
see `CLAUDE.md` — not repeated here.

## Pre-session prep (before arriving)

- [ ] Confirm with Tsaqif what exactly is blocking LIDAR/RealSense setup (driver, power, USB, mounting).
- [ ] Confirm Axiomtek SBC CPU variant (x86_64 vs ARM) — still open in CLAUDE.md, gates ROS2/RealSense package installs.
- [ ] `git pull` on whatever machine will be used in the lab.
- [ ] Battery charged; XT60 and lever-nut connections checked.

## Step by step, in order

1. **Diagnose and fix the LIDAR/RealSense issue first.** Tsaqif leads. Don't start SBC integration work while this is unresolved.
2. **Verify each sensor independently:**
   - [ ] LIDAR: `ros2 launch rplidar_ros rplidar_a1_launch.py`, then `ros2 topic echo /scan` — confirm real, non-garbage ranges.
   - [ ] RealSense: confirm it enumerates and streams before touching any ROS2 wrapper.
3. **Confirm SBC CPU variant** if still unresolved, and update the open question in CLAUDE.md once known.
4. **Rehearse Layer 2 on real hardware** (currently unconfirmed):
   - [ ] With LIDAR live, run `obstacle_detector_node.py`; confirm it writes `'S'` to the Arduino on clear→blocked.
   - [ ] Confirm the Arduino sketch halts motors on `'S'`, and `'G'` clears it without auto-resuming.
5. **Stress-test the shared USB hub** (RealSense + LIDAR both connect through one hub on the SBC) — with both streaming, check `lsusb` / `dmesg` for enumeration drops or bandwidth issues before blaming a driver.
6. **Arduino → SBC integration:**
   - [ ] Move the Arduino's USB from laptop to SBC.
   - [ ] Set up SSH into the SBC as the default way of working, so nobody needs to be physically hands-on the robot to test.
   - [ ] Confirm serial commands (`F100`, `S`, `G`) still work identically over the new connection.
7. **End-to-end integration test** — the milestone for the session: object enters the LIDAR cone → `'S'` sent → Arduino stops, with the robot actually driving, not idle.
8. **Wrap-up (don't skip):**
   - [ ] Update CLAUDE.md's "Current status" and "Known open questions" with whatever got confirmed or is still stuck.
   - [ ] Commit on the right person's branch (`person-b-arduino` / `person-c-perception`), small and reviewed. Merge to `main` only once it ran clean on hardware.
   - [ ] Replace this file's contents with the plan for the *next* session (or delete it if nothing further is queued).

## Explicitly out of scope for this session

Nav2 / SLAM / full autonomy (Layer 3 stretch) — not worth touching until step 7 above is solid.
