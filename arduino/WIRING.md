# Wiring reference — AMR motion control

Bench reference for Person B (Arduino / motion control). Pin numbers are the
source of truth from `motor_control.ino` / `main_robot.ino` — if code and this
doc ever disagree, the code wins and this doc should be fixed. See `CLAUDE.md`
(Critical hardware facts) for the power-distribution and safety reasoning.

Status legend: ✅ done · ⬜ to do / verify · 🚫 blocked

## Pin map (from the sketches — canonical)

| Corner | Label | Driver / channel | IN pins (Mega) | Motor OUT terminals |
|---|---|---|---|---|
| Front-Left  | FL | Driver 1, ch A | IN1→**D2**, IN2→**D3** | D1 OUT1/OUT2 |
| Front-Right | FR | Driver 1, ch B | IN3→**D4**, IN4→**D5** | D1 OUT3/OUT4 |
| Rear-Left   | RL | Driver 2, ch A | IN1→**D6**, IN2→**D7** | D2 OUT1/OUT2 |
| Rear-Right  | RR | Driver 2, ch B | IN3→**D8**, IN4→**D9** | D2 OUT3/OUT4 |

All 8 IN pins are PWM-capable (`~` on the silkscreen). Speed is set with
`analogWrite()` directly on the active IN pin — these boards have no separate
PWM/enable pin. **A wrong spin direction is fixed by swapping that motor's two
OUT leads at the screw terminal, never by flipping signs in code.**

Motor 6-pin JST connector colours: red = motor+, white = motor−,
blue = encoder VCC, black = encoder GND, yellow = phase A, green = phase B.

## Wiring plan by electrical domain

### ① Battery / motor power (high current)
- ✅ Battery XT60 → lever-nut distribution block input
- ✅ Block **Channel 1** (blue + / orange −) → **Driver 1** VCC/GND (front)
- ✅ Block **Channel 2** (blue + / orange −) → **Driver 2** VCC/GND (rear)
- ✅ Block output line → **SZDULI** 19V converter input → SBC
- ✅ Motor OUT leads per the pin-map table above
- ⬜ Keep OUT leads **swappable** at the screw terminals (for direction fixes)

Blue levers = battery **positive** rail, orange levers = battery **negative**
rail (internally isolated bus bars). The block only distributes — it does not
regulate — so every output sits at raw battery voltage. **No breadboard is
anywhere on the power path.**

### ② Logic signals (Mega → drivers, low current)
- ✅ Driver 1 IN1–IN4 → **D2, D3, D4, D5**
- ✅ Driver 2 IN1–IN4 → **D6, D7, D8, D9**
- ⬜ Both driver logic **5V+ → Mega 5V**, **GND → Mega GND**, via a small
  distribution point (terminal block / soldered bus) — don't cram 4+ jumpers
  into one Mega pin, and leave room for encoder VCC/GND.

Note: driver *logic* 5V/GND (from the Mega) is a **separate power domain** from
driver *motor* VCC/GND (from the block). Keep them straight when troubleshooting.

### ③ Common ground (critical)
- ⬜ **Mega GND ↔ block orange (battery −) rail** — verify with a multimeter.
  Without it the motor-control signals have no shared reference.

### ④ Encoders (real pin-out, wired 2026-07-16 — see `encoder_test.ino`)
Only 6 Mega pins support true hardware interrupts (D2, D3, D18, D19, D20,
D21), and D2/D3 are already taken by the FL motor — so only FL and FR get
full hardware-interrupt pairs. RL and RR instead use the Mega's PCINT0 group
(raw AVR registers, not a library — see `encoder_test.ino`). Corrects an
earlier plan here that assumed only phase A needed an interrupt pin and that
all 4 corners would fit on D18–D21.

| Corner | Phase A — yellow | Phase B — green | VCC — blue | GND — black |
|---|---|---|---|---|
| FL | **D18** (hw interrupt) | **D19** | Mega 5V bus | Mega GND bus |
| FR | **D20** (hw interrupt) | **D21** | Mega 5V bus | Mega GND bus |
| RL | **D11** (pin-change interrupt) | **D24** | Mega 5V bus | Mega GND bus |
| RR | **D12** (pin-change interrupt) | **D25** | Mega 5V bus | Mega GND bus |

- ✅ **All 4 encoders** — wired per the table above, matches `encoder_test.ino`
  ground truth. Confirm live counts on each wheel via `encoder_test.ino`
  before trusting calibration.
- ⚠️ **Encoder VCC (blue) goes to the Mega 5V bus, never to the lever-nut
  block.** The block outputs raw battery voltage (7.4–8.4V) — that will fry
  the encoder's Hall-sensor electronics, which expect 5V logic.
- ✅ **Firmware note:** `main_robot.ino` now reads all 4 wheels' counts (ported
  from `encoder_test.ino`, same PCINT0 handling for RL/RR). Only FL's count
  drives the `F<cm>` stop condition — the other 3 are diagnostics-only, since
  `COUNTS_PER_CM` was calibrated against FL alone.

### ⑤ Arduino power / comms
- ⬜ Mega **USB → SBC (or laptop)** — powers the Mega and carries the serial
  protocol (`F<cm>` / `S` / `G`). **The Mega is NOT powered from the block.**

### Motor spin-direction — resolved
- ✅ **Motor spin-direction verified (2026-07-17)** — forward/reverse/strafe/
  rotate all confirmed correct per-wheel via `motor_control.ino`'s single-motor
  ID test and combined movement keys, plus correct motion on the ground. No OUT
  lead swaps were needed. This supersedes the earlier note here that direction
  verification was blocked on a mechanical shaft-to-wheel coupling fix.

## Suggested arrange order
1. **Common ground first** (③) — bond Mega GND to the orange rail, confirm continuity.
2. **Logic 5V/GND distribution** (②) — set up the shared feed point.
3. **Dress the 8 IN lines** (②) into a tidy D2–D9 bundle.
4. **Route/secure the power domain** (①) — block, both channels, SZDULI tap.
5. **Add the FL encoder** (④).
6. **Continuity / short check, then power up wheels-off** to confirm the rearrange.

## Clean-wiring tips
Aim for **tidy + reliable + re-openable** — you're still iterating (direction
unverified, encoders to add), so this isn't a final sealed loom yet.

- **Separate the two domains physically** — don't bundle high-current motor-power
  wires tight against logic/encoder signal lines (noise + heat). Keep encoder
  A/B lines especially away from motor-power runs.
- **Twist your pairs** — each motor's OUT leads, the encoder bundle, the power
  +/− runs. Tidier and quieter (less radiated noise).
- **Label both ends of every cable** (e.g. "FL / IN D2"), not just the motors.
- **Velcro straps over zip ties for now** — you'll be back in to swap OUT leads
  and add encoders; save permanent zip ties for the final pass.
- **Leave service loops** — a little slack at every connector so reseating or
  swapping a lead doesn't pull on a terminal or solder joint.
- **Ferrule or tin stranded wire** into screw terminals so stray strands can't
  bridge to the neighbour and short. Check the lever-nut block's wire rating.
- **Strain-relief near connectors** so robot vibration can't back out a screw
  terminal or fatigue a joint over time.
- **Keep wires clear of the wheels/rollers**; route along chassis structure.
- **Keep the XT60 (power kill) and the Mega USB port accessible.**
- **Don't over-cinch ties** — crushing signal wires causes intermittent faults.
