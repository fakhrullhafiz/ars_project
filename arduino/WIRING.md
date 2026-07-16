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

### ④ Encoders (pin plan finalized 2026-07-16 — bring up one at a time)
Only phase A needs a true hardware-interrupt pin (`attachInterrupt()` in the
code); phase B is read with a plain `digitalRead()` inside that ISR, so it can
go on any free digital pin. That means all 4 corners fit on the Mega's 4
remaining interrupt pins (D18–D21, since D2/D3 are taken by the FL motor) —
corrects an earlier note here that assumed both channels needed interrupt
pins and that only 2 encoders (FL+FR) would fit.

| Corner | Phase A — yellow (interrupt) | Phase B — green (direction) | VCC — blue | GND — black |
|---|---|---|---|---|
| FL | **D18** | **D22** | Mega 5V bus | Mega GND bus |
| FR | **D19** | **D23** | Mega 5V bus | Mega GND bus |
| RL | **D20** | **D24** | Mega 5V bus | Mega GND bus |
| RR | **D21** | **D25** | Mega 5V bus | Mega GND bus |

- ⬜ **FL encoder** — wire per the table above, verify with `encoder_test.ino`
  (default pins already match FL) before moving on.
- ⬜ **FR / RL / RR encoders** — same pattern, one corner at a time. Edit
  `encoder_test.ino`'s `ENC_A_PIN`/`ENC_B_PIN` to that corner's row to verify
  each before moving to the next; don't wire and test all 4 untested at once.
- ⚠️ **Encoder VCC (blue) goes to the Mega 5V bus, never to the lever-nut
  block.** The block outputs raw battery voltage (7.4–8.4V) — that will fry
  the encoder's Hall-sensor electronics, which expect 5V logic.
- 🚧 **Firmware note:** `main_robot.ino` currently only reads FL's counts (one
  `volatile long encoderCount`, one ISR). Wiring all 4 now is safe — nothing
  is damaged by idle connections — but RL/RR/FR won't produce live counts in
  `main_robot.ino` until the code is extended with 4 separate counters/ISRs.
  That's a firmware task for after wiring, not a blocker to wiring.

### ⑤ Arduino power / comms
- ⬜ Mega **USB → SBC (or laptop)** — powers the Mega and carries the serial
  protocol (`F<cm>` / `S` / `G`). **The Mega is NOT powered from the block.**

### 🚫 Blocked (not a wiring task)
- Motor spin-**direction** verification (forward/reverse/strafe/rotate) waits on
  the mechanical shaft-to-wheel coupling fix (motor shaft dia. vs wheel bore) —
  Person A / Haikal's scope. Confirm directions with wheels on, then swap OUT
  leads for any wrong wheel.

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
