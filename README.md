# ars_project

Autonomous Mobile Robot (AMR) — 4-week group project. Mecanum-wheel platform with Arduino-level motion control and RPLIDAR-based obstacle avoidance, with an optional ROS2/SBC stretch layer.

**Baseline goal:** robot moves reliably from Point A to Point B.
**Stretch goal:** LIDAR-based obstacle avoidance, and (time permitting) ROS2 Nav2 / RealSense integration.

Full project plan, timeline, and task breakdown live in Google Drive (see [Where things live](#where-things-live) below) — this repo holds code only.

---

## Team & ownership

| Person | Name | Owns | Folder(s) |
|---|---|---|---|
| Person A | Haikal | Mechanical, power, 3D printing, wiring | *(no code folder — CAD lives in Drive)* |
| Person B | Pekol (fakhrullhafiz) | Low-level motion control | `arduino/` |
| Person C | Tsaqif | Perception, navigation, ROS2 | `ros2_workspace/` |

Each folder is owned by one person day-to-day, but anyone can open a PR against any folder — see [Branching](#branching--workflow) below.

---

## Repo structure

This will grow as work progresses — add new sketch/package folders as needed, but keep the two-folder top-level split (`arduino/` vs `ros2_workspace/`) so it's always obvious which subsystem a piece of code belongs to.

---

## Where things live

Code-only in this repo. Everything else is in the shared Google Drive `ARS_Project` folder:

- **Documentation/** — project plan, component documentation, report draft
- **CAD/** — Bambu Studio project files and exported STLs for printed parts
- **Photos/** — assembly progress, organized by week
- **Report_Draft** — the actual written report (Google Doc, exported to .docx at the end)

If you're looking for the *why* behind a design decision (e.g. why the motor PWM is capped, why a part was reprinted), check `Documentation/` in Drive first — that's the narrative record. This repo is the working code.

---

## Branching & workflow

- `main` is always the last known-working state. Don't push broken code directly to `main`.
- Each person works primarily on their own branch:
  - `person-a-mechanical` *(rarely used for code, but available if Person A scripts anything — e.g. calibration helpers)*
  - `person-b-arduino`
  - `person-c-perception`
- Merge into `main` only once something has been tested on actual hardware, not just "it compiles." A sketch that compiles but hasn't driven the real motors is not yet `main`-ready.
- Small, frequent commits beat one giant end-of-week commit — easier to tell what broke and when.
- If using Claude Code to generate or modify code, review the diff before committing. Don't blindly accept generated changes on hardware-control code (motor PWM limits, safety stops) without a human read-through.

---

## Getting started (any teammate, any machine)

```bash
git clone <repo-url>
cd ars_project
```

**Arduino work:** open the relevant sketch folder under `arduino/` in the Arduino IDE.

**ROS2 work:**
```bash
cd ros2_workspace
colcon build
source install/setup.bash
```

See `CLAUDE.md` for fuller project context if working with Claude Code.

---

## Status

Update this section weekly so anyone opening the repo can see where things stand at a glance.

- **Week 1:** Repo scaffolded (README, CLAUDE.md, folder layout). Starter sketches added: `motor_control.ino`, `encoder_test.ino`, `main_robot.ino`, and the `obstacle_avoidance` ROS2 package. Battery confirmed as 2S LiPo (7.4V nominal/8.4V full), `MAX_PWM` set to 180. Single-motor ID test mode and `arduino/WIRING.md` bench reference added.
- **Week 2:** No code changes this week — team focus was elsewhere (mechanical/CAD, planning). Encoder rewire and 4-wheel integration work picked up mid-July.
- **Week 3:** Layers 0–1 (open-loop motor control + closed-loop encoder driving) fully validated on hardware — all 4 wheels, `MAX_PWM=180` thermal-checked, `COUNTS_PER_CM` calibrated (see CLAUDE.md). Layer 2 (RPLIDAR obstacle-stop) is written but not yet rehearsed on real hardware — blocked on a LIDAR/RealSense setup issue, to be resolved in the lab. Next session: unblock sensors, rehearse Layer 2 end-to-end, then begin Arduino→Axiomtek SBC integration. Step-by-step plan in [`NEXT_SESSION.md`](NEXT_SESSION.md).
- **Week 4:** _(update here)_
