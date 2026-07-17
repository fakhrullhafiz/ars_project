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
// 180/255 (~70%) matches the confirmed 2S battery: 6.0V motor rating / 8.4V
// full-charge pack voltage is ~71%. See motor_control.ino and CLAUDE.md for
// the full reasoning -- do not raise further without motor temp testing.
// Was temporarily 100 for the COUNTS_PER_CM sanity check (F100 test,
// 2026-07-17) -- that test passed (actual stop landed close to the 100cm
// mark), so restored to 180. Expect somewhat more coasting overshoot at
// this speed than what was observed during the slower sanity check, since
// this sketch has no braking (see setup()/loop() comments).
const int MAX_PWM = 180;  // out of 255 (~70%)
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
enum RobotState { IDLE, DRIVING, EMERGENCY_STOPPED };
RobotState state = IDLE;
long targetCounts = 0;

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
  Serial.println(F("main_robot ready. Commands: F<cm>  S  G"));
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
