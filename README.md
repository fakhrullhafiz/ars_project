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

- **Week 1:** _(update here)_
- **Week 2:** _(update here)_
- **Week 3:** _(update here)_
- **Week 4:** _(update here)_
