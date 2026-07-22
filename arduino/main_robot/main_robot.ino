/*
  main_robot.ino
  ----------------------------------------------------------------------------
  Layer 1 (closed-loop "drive to distance") with a Layer 2 extension point for
  obstacle-stop commands from the RPLIDAR/perception side.

  PREREQUISITES — confirm both of these work BEFORE using this sketch:
    - motor_control.ino: all 4 wheels respond correctly to direction/speed
    - encoder_test.ino:  you have a calibrated COUNTS_PER_CM value (see below)

  *** Update COUNTS_PER_CM below with YOUR calibrated value from encoder_test.ino
  before relying on this for distance accuracy. The placeholder value here is
  a rough guess and will NOT be accurate for your actual wheels. ***

  SERIAL COMMAND PROTOCOL (interface for Layer 2 / perception side):
    Over USB serial (115200 baud), this sketch accepts:
      'F' <distance_cm>  -- drive forward N cm then stop  (e.g. "F100\n")
      'S'                -- emergency stop, immediately zero all motors
      'G'                -- resume/go (clears a stop triggered by 'S')
    Intentionally simple text protocol so Person C's RPLIDAR code can send
    plain strings over serial. Do not change command characters unilaterally --
    both sides depend on this exact format.

  MANUAL TELEOP (added for wheel-odometry/SLAM bring-up, 2026-07-22):
    'w'/'s'/'a'/'d'/'q'/'e'/'x' -- forward/reverse/strafe-left/strafe-right/
    rotate-left/rotate-right/stop, ported from motor_control.ino's confirmed
    -correct mecanum direction logic (2026-07-17 spin-direction test). Added
    here (not just left in motor_control.ino) because this sketch is the only
    one that also tracks all 4 encoders -- driving with motor_control.ino
    would give ROS2 nothing to compute odometry from. Ignored while
    EMERGENCY_STOPPED (send 'G' first).

  ENCODER TELEMETRY (added 2026-07-22, for ROS2-side wheel_odometry_node):
    Every ~50ms this sketch prints one extra line:
      "E,<flCount>,<frCount>,<rlCount>,<rrCount>,<millis>\n"
    The 'E,' prefix lets a serial reader on the ROS2 side pick this line out
    from the other human-readable status lines above. Counts are raw --
    ROS2 side is responsible for the documented per-side sign correction and
    the COUNTS_PER_CM conversion (see CLAUDE.md's encoder sign-convention
    note: right-side wheels count ++ forward, left-side wheels count --
    forward).

  WIRING -- motor pins match motor_control.ino exactly:
    IN pins: ~D2 through ~D9 on right-side header (PWM-capable)

  4-ENCODER PIN PLAN (matches physical wiring done 2026-07-16 -- see CLAUDE.md
  and arduino/encoder_test.ino; all 4 wheels now wired into this sketch's
  logic, 2026-07-17):
    Only D2, D3, D18, D19, D20, D21 are true hardware-interrupt pins on the
    Mega, and D2/D3 are already taken by the FL motor, so only FL and FR get
    full interrupt-pin pairs. RL and RR use the Mega's PCINT0 group instead
    (raw AVR registers, same approach as encoder_test.ino):
      FL: A=D18, B=D19   (hardware interrupt)
      FR: A=D20, B=D21   (hardware interrupt)
      RL: A=D11, B=D24   (pin-change interrupt)
      RR: A=D12, B=D25   (pin-change interrupt)
    IMPORTANT -- only flCount drives the 'F<cm>' stop condition below.
    frCount/rlCount/rrCount are tracked and printed for diagnostics only
    (e.g. spotting a wheel that's under/over-rotating vs. the others) --
    they do NOT affect drive timing or distance accuracy. COUNTS_PER_CM was
    measured on FL alone, so only FL's count is calibrated; averaging in
    the other 3 wheels would need its own fresh hardware validation first.
*/

// ---- Calibration -- measured via encoder_test.ino (2026-07-17) ----
// Front-left wheel, 1 hand-rotated revolution = 70 counts (magnitude; FL reads
// negative on forward per the documented sign convention, doesn't matter here
// since loop() compares abs(flCount)). Wheel diameter
// 97mm -> circumference = pi * 97mm =~ 304.73mm = 30.473cm.
// COUNTS_PER_CM = 70 / 30.473 =~ 2.297
const float COUNTS_PER_CM = 2.297;

// ---- Safety PWM ceiling -- keep in sync with motor_control.ino ----
// RAISED TO 255 (100%), 2026-07-22, explicit user request/acknowledged
// tradeoff -- NOT the previously-validated safe value. Full chassis assembly
// (all components mounted -- SBC, RealSense, LIDAR, wiring) made the robot
// heavy enough that it was bogging down under sustained load at the old
// 180 (~70%) ceiling even on a freshly-charged battery, i.e. a genuine
// torque shortfall, not a battery-sag or startup-stiction issue.
// 180/255 (~70%) was originally chosen to keep motor voltage at/below the
// JGB37-520's 6.0V rating against the 8.4V-max 2S pack (6.0/8.4 ~= 71%) --
// see CLAUDE.md's Critical Hardware Facts. Running at 255 removes that
// margin entirely: the motors now see full battery voltage continuously
// while driving (not just briefly), well above their rated 6.0V.
// The 2026-07-17 thermal check that validated safe operation was done at
// 180 and pre-full-assembly weight -- it does NOT cover this value or this
// load. Watch motor casing temperature closely during testing; drop this
// back down if anything runs hot.
const int MAX_PWM = 255;  // out of 255 (100%) -- exceeds motor voltage rating, see above
const int MIN_PWM = 60;   // below this the motor may not overcome static friction

// ---- Motor pins -- must match motor_control.ino exactly ----
const int FL_IN1 = 2,  FL_IN2 = 3;   // Front-Left  (~D2, ~D3)
const int FR_IN1 = 4,  FR_IN2 = 5;   // Front-Right (~D4, ~D5)
const int RL_IN1 = 6,  RL_IN2 = 7;   // Rear-Left   (~D6, ~D7)
const int RR_IN1 = 8,  RR_IN2 = 9;   // Rear-Right  (~D8, ~D9)

// ---- Encoder pins, all 4 wheels -- see 4-encoder pin plan in header ----
// D18 = TX1 / D19 = RX1 on the board label -- usable as digital I/O when
// Serial1 is not in use, which is the case here. Naming matches
// encoder_test.ino, the proven reference implementation this was ported from.
const int FL_A = 18, FL_B = 19;  // hardware interrupt
const int FR_A = 20, FR_B = 21;  // hardware interrupt
const int RL_A = 11, RL_B = 24;  // pin-change interrupt -- RL_A = Mega pin 11 = AVR PB5 = PCINT5
const int RR_A = 12, RR_B = 25;  // pin-change interrupt -- RR_A = Mega pin 12 = AVR PB6 = PCINT6

// flCount is the only one used for the drive-to-distance stop condition
// (see loop()). frCount/rlCount/rrCount are diagnostics-only -- see the
// IMPORTANT note in the header comment above before wiring them into any
// stop/steering logic.
volatile long flCount = 0, frCount = 0, rlCount = 0, rrCount = 0;
volatile uint8_t lastPINB = 0;

// ---- State machine ----
// MANUAL_DRIVE is distinct from DRIVING: DRIVING is the F<cm> auto-stop-at-
// -target mode (see loop()'s abs(flCount) >= targetCounts check); MANUAL_DRIVE
// is open-loop teleop with no target, added for driving around during
// wheel-odometry/SLAM bring-up.
enum RobotState { IDLE, DRIVING, MANUAL_DRIVE, EMERGENCY_STOPPED };
RobotState state = IDLE;
long targetCounts = 0;

// ---- Encoder telemetry timer (see header comment) ----
const unsigned long TELEMETRY_INTERVAL_MS = 50;
unsigned long lastTelemetryMs = 0;

void setup() {
  Serial.begin(115200);

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

  pinMode(FL_A, INPUT_PULLUP); pinMode(FL_B, INPUT_PULLUP);
  pinMode(FR_A, INPUT_PULLUP); pinMode(FR_B, INPUT_PULLUP);
  pinMode(RL_A, INPUT_PULLUP); pinMode(RL_B, INPUT_PULLUP);
  pinMode(RR_A, INPUT_PULLUP); pinMode(RR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_A), handleFL, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_A), handleFR, RISING);

  // RL/RR: pin-change interrupt on PCINT0 group (covers D10-D13, D50-D53),
  // same approach as encoder_test.ino -- no true hardware-interrupt pins
  // left free for them.
  lastPINB = PINB;
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT5) | (1 << PCINT6);

  stopAll();
  Serial.println(F("main_robot ready. Commands: F<cm>  S  G  |  teleop: w a s d q e x"));
}

void loop() {
  handleSerialCommands();

  if (state == DRIVING) {
    if (abs(flCount) >= targetCounts) {
      stopAll();
      state = IDLE;
      Serial.println(F("Target reached, stopped."));
      // Diagnostics only -- FL alone decided the stop above. Compare these
      // 4 counts to spot a wheel that's under/over-rotating vs. the others;
      // see the IMPORTANT note in the header comment before using them for
      // anything beyond diagnostics.
      Serial.print(F("Encoder counts -- FL:")); Serial.print(flCount);
      Serial.print(F(" FR:")); Serial.print(frCount);
      Serial.print(F(" RL:")); Serial.print(rlCount);
      Serial.print(F(" RR:")); Serial.println(rrCount);
    }
    // NOTE: open-loop speed with closed-loop distance cutoff -- does not yet
    // correct for left/right drift. frCount/rlCount/rrCount above are a step
    // toward that; adjusting individual PWM based on them is still a future
    // extension, not implemented here.
  }

  unsigned long now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    noInterrupts();
    long fl = flCount, fr = frCount, rl = rlCount, rr = rrCount;
    interrupts();
    Serial.print(F("E,"));
    Serial.print(fl); Serial.print(F(","));
    Serial.print(fr); Serial.print(F(","));
    Serial.print(rl); Serial.print(F(","));
    Serial.print(rr); Serial.print(F(","));
    Serial.println(now);
  }
}

void handleSerialCommands() {
  if (Serial.available() <= 0) return;

  char cmd = Serial.read();

  if (cmd == 'F') {
    float distanceCm = Serial.parseFloat();
    if (distanceCm > 0) {
      noInterrupts();
      flCount = 0; frCount = 0; rlCount = 0; rrCount = 0;
      interrupts();
      targetCounts = (long)(distanceCm * COUNTS_PER_CM);
      state = DRIVING;
      driveForward();
      Serial.print(F("Driving forward "));
      Serial.print(distanceCm);
      Serial.println(F(" cm"));
    }
  } else if (cmd == 'S') {
    // Emergency stop -- Person C sends 'S' the moment an obstacle is detected.
    stopAll();
    state = EMERGENCY_STOPPED;
    Serial.println(F("EMERGENCY STOP"));
  } else if (cmd == 'G') {
    // Clear emergency stop. Does NOT auto-resume previous drive -- re-issuing
    // an 'F' command is intentional so the perception side re-validates first.
    if (state == EMERGENCY_STOPPED) {
      state = IDLE;
      Serial.println(F("Cleared emergency stop, ready for new command."));
    }
  } else if (cmd == 'w' || cmd == 's' || cmd == 'a' || cmd == 'd' ||
             cmd == 'q' || cmd == 'e' || cmd == 'x') {
    // Manual teleop -- ignored during EMERGENCY_STOPPED, same as 'F' should
    // be but isn't (pre-existing gap in the F handler above, not touched
    // here to keep this diff scoped to the teleop/telemetry addition).
    if (state == EMERGENCY_STOPPED) return;
    switch (cmd) {
      case 'w': driveForward();  state = MANUAL_DRIVE; Serial.println(F("forward"));      break;
      case 's': driveReverse();  state = MANUAL_DRIVE; Serial.println(F("reverse"));      break;
      case 'a': strafeLeft();    state = MANUAL_DRIVE; Serial.println(F("strafe left"));  break;
      case 'd': strafeRight();   state = MANUAL_DRIVE; Serial.println(F("strafe right")); break;
      case 'q': rotateLeft();    state = MANUAL_DRIVE; Serial.println(F("rotate left"));  break;
      case 'e': rotateRight();   state = MANUAL_DRIVE; Serial.println(F("rotate right")); break;
      case 'x': stopAll();       state = IDLE;          Serial.println(F("stop"));         break;
    }
  }
}

// speed: -255..255. Active IN pin gets analogWrite for speed; other held LOW.
void setMotor(int in1, int in2, int speed) {
  int clamped = constrain(abs(speed), 0, MAX_PWM);
  if (clamped > 0 && clamped < MIN_PWM) clamped = MIN_PWM;

  if (speed > 0) {
    analogWrite(in1, clamped);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    analogWrite(in2, clamped);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void driveForward() {
  setMotor(FL_IN1, FL_IN2,  MAX_PWM);
  setMotor(FR_IN1, FR_IN2,  MAX_PWM);
  setMotor(RL_IN1, RL_IN2,  MAX_PWM);
  setMotor(RR_IN1, RR_IN2,  MAX_PWM);
}

void stopAll() {
  setMotor(FL_IN1, FL_IN2, 0);
  setMotor(FR_IN1, FR_IN2, 0);
  setMotor(RL_IN1, RL_IN2, 0);
  setMotor(RR_IN1, RR_IN2, 0);
}

// ---- Teleop-only movement functions, ported verbatim from motor_control.ino
// (2026-07-17 spin-direction test, all 4 wheels + strafe/rotate confirmed
// correct on hardware) -- driveForward() above already matched this pattern
// and is reused as-is for 'w'.
void driveReverse() {
  setMotor(FL_IN1, FL_IN2, -MAX_PWM);
  setMotor(FR_IN1, FR_IN2, -MAX_PWM);
  setMotor(RL_IN1, RL_IN2, -MAX_PWM);
  setMotor(RR_IN1, RR_IN2, -MAX_PWM);
}

void strafeLeft() {
  setMotor(FL_IN1, FL_IN2, -MAX_PWM);
  setMotor(FR_IN1, FR_IN2,  MAX_PWM);
  setMotor(RL_IN1, RL_IN2,  MAX_PWM);
  setMotor(RR_IN1, RR_IN2, -MAX_PWM);
}

void strafeRight() {
  setMotor(FL_IN1, FL_IN2,  MAX_PWM);
  setMotor(FR_IN1, FR_IN2, -MAX_PWM);
  setMotor(RL_IN1, RL_IN2, -MAX_PWM);
  setMotor(RR_IN1, RR_IN2,  MAX_PWM);
}

void rotateLeft() {
  setMotor(FL_IN1, FL_IN2, -MAX_PWM);
  setMotor(FR_IN1, FR_IN2,  MAX_PWM);
  setMotor(RL_IN1, RL_IN2, -MAX_PWM);
  setMotor(RR_IN1, RR_IN2,  MAX_PWM);
}

void rotateRight() {
  setMotor(FL_IN1, FL_IN2,  MAX_PWM);
  setMotor(FR_IN1, FR_IN2, -MAX_PWM);
  setMotor(RL_IN1, RL_IN2,  MAX_PWM);
  setMotor(RR_IN1, RR_IN2, -MAX_PWM);
}

// Quadrature direction logic matches encoder_test.ino: if B is HIGH when A
// rises, one direction; if B is LOW, the other.
void handleFL() { if (digitalRead(FL_B) == HIGH) flCount++; else flCount--; }
void handleFR() { if (digitalRead(FR_B) == HIGH) frCount++; else frCount--; }

// Fires on ANY change on PB0-PB7 (D10-D13, D50-D53) -- must check which of
// our two watched pins actually changed, and that it was a rising edge, to
// match the RISING-only behavior used for FL/FR above. Identical to
// encoder_test.ino's proven implementation.
ISR(PCINT0_vect) {
  uint8_t pinb = PINB;
  uint8_t changed = pinb ^ lastPINB;

  if ((changed & (1 << PCINT5)) && (pinb & (1 << PCINT5))) {  // D11 RL rising
    if (digitalRead(RL_B) == HIGH) rlCount++; else rlCount--;
  }
  if ((changed & (1 << PCINT6)) && (pinb & (1 << PCINT6))) {  // D12 RR rising
    if (digitalRead(RR_B) == HIGH) rrCount++; else rrCount--;
  }
  lastPINB = pinb;
}
