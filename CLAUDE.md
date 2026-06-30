# CLAUDE.md

Context for Claude Code when working in this repo. Read this before making changes — especially anything touching motor control or power.

## What this project is

A 3-person, 4-week university robotics project: a mecanum-wheel autonomous mobile robot (AMR). The graded bar is low (move from Point A to Point B), so this repo deliberately prioritizes a working baseline over feature completeness. Don't add complexity that isn't asked for in a given task — see "Priority layers" below before suggesting scope expansions.

Full hardware specs live in `Documentation/robotics_component_documentation.md` in the shared Google Drive (not in this repo — this repo is code-only). Ask the user for the relevant section if you need exact pin numbers, voltage specs, or part numbers rather than guessing.

## Priority layers (do not skip ahead)

1. **Layer 0 — Baseline:** Arduino Mega drives all 4 motors via the MC33886VW drivers, robot physically moves forward/turns. This alone satisfies the assignment.
2. **Layer 1 — Reliable navigation:** encoder feedback (JGB37-520 magnetic encoders) used for closed-loop "drive to distance" / "turn to angle", so A→B is repeatable, not timing-based.
3. **Layer 2 — Stretch:** RPLIDAR-based obstacle detection layered on top of Layer 1.
4. **Layer 3 — Optional stretch:** Axiomtek SBC + RealSense D455 + ROS2 Nav2/SLAM. Only relevant once Layers 0–2 are solid and rehearsed.

If asked to work on something from a higher layer, check whether the layer below it is confirmed working first. If unclear, ask rather than assuming it's done.

## Critical hardware facts — do not regenerate these from training data, treat as ground truth

- **Motors (JGB37-520):** rated 6.0V DC.
- **Battery:** LiPo, either 3S (11.1V nominal / 12.6V max) or 4S (14.8V nominal / 16.8V max) — config TBD, confirm with team before assuming.
- **This is a voltage mismatch by design, not a bug.** The MC33886VW driver passes battery voltage straight through to the motors — it does not regulate it down. The fix is software: cap PWM duty cycle (current working assumption: 40–50% ceiling, tune based on real motor temperature during testing) rather than running motors at full duty cycle. **Never write or suggest motor control code that defaults to 100% PWM duty cycle** without an explicit, deliberate reason — this risks motor damage given the voltage mismatch.
- **SZDULI Y2-K101908 converter:** steps battery voltage to a clean 19V rail for the Axiomtek SBC, rated up to 152W. Not a bottleneck — don't spend effort here unless explicitly asked.
- **RPLIDAR A1 pinout:** VCC (+5V), GND, TX (3.3V TTL), RX (3.3V TTL), M_EN (motor enable/PWM).
- **Arduino Mega 2560:** ATmega2560, 5V logic, 54 digital I/O pins, hardware Serial0–3, I2C on pins 20/21.

If any of these facts appear to conflict with something in code or in a teammate's comment, flag the discrepancy to the user rather than silently picking one.

## Repo layout
CAD files, the project plan, the report draft, and assembly photos are in Google Drive, not here. If you need them, ask the user to paste/upload the relevant content rather than assuming they're accessible from this repo.

## Working conventions

- **Test on hardware before suggesting a merge to `main`.** "Compiles successfully" is not the same as "drives the real motors correctly" — say this explicitly when handing code back, don't imply it's done.
- **Small, reviewable diffs.** Avoid large multi-file rewrites in one pass unless asked; this is a team of beginners and large diffs are hard for them to review and learn from.
- **Always explain *why*, briefly, for anything touching motor PWM limits, encoder calibration constants, or safety stop logic** — these are the values most likely to need re-tuning by a teammate who isn't deeply familiar with the code.
- **Don't introduce ROS2 into the Arduino-side code, and don't introduce Arduino-specific assumptions into the ROS2 workspace.** The two communicate over a simple serial interface (see `ars_project/README.md` for current protocol status) — keep that boundary clean.
- **Ask before adding new dependencies/libraries**, especially on the ROS2 side — install footprint and version compatibility (this team uses Ubuntu 24.04, ROS2 Humble or Jazzy — confirm which before assuming) matters for a project where setup time is scarce.

## Known open questions (update as resolved)

- [ ] Confirmed battery S-count (3S vs 4S) and whether the existing pack is healthy (see Drive Documentation, Section 2 of the project plan).
- [ ] Confirmed Axiomtek SBC CPU variant (i3/i5/i7/Celeron).
- [ ] ROS2 distro decision (Humble vs Jazzy) finalized.
- [ ] Serial protocol between Arduino and RPLIDAR-side obstacle logic — exact byte format not yet defined as of repo creation.

Keep this list current. A stale "known open questions" list is worse than none — if something here gets resolved, delete it or check it off rather than leaving it stale for the next session to trip over.
