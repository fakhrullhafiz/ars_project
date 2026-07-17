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
    Encoder (front-left only, this sketch): D18 (interrupt, phase A), D19 (phase B)

  4-ENCODER PIN PLAN (matches physical wiring done 2026-07-16 -- see CLAUDE.md
  and arduino/encoder_test.ino; only FL is wired into this sketch's logic so
  far, see arduino/WIRING.md for full wiring status):
    Only D2, D3, D18, D19, D20, D21 are true hardware-interrupt pins on the
    Mega, and D2/D3 are already taken by the FL motor, so only FL and FR get
    full interrupt-pin pairs. RL and RR use the Mega's PCINT0 group instead
    (raw AVR registers, see encoder_test.ino) -- not implemented in this
    sketch yet:
      FL: A=D18, B=D19   (wired into this sketch)
      FR: A=D20, B=D21   (reserved, not yet read by this code)
      RL: A=D11, B=D24   (pin-change interrupt; reserved, not yet read by this code)
      RR: A=D12, B=D25   (pin-change interrupt; reserved, not yet read by this code)
    Reading all 4 simultaneously needs 4 separate counters + 4 ISRs (and, for
    RL/RR, PCINT0 handling like encoder_test.ino's) -- a firmware extension
    for later, not implemented here yet.
*/

// ---- Calibration -- measured via encoder_test.ino (2026-07-17) ----
// Front-left wheel, 1 hand-rotated revolution = 70 counts (magnitude; FL reads
// negative on forward per the documented sign convention, doesn't matter here
// since handleSerialCommands() compares abs(encoderCount)). Wheel diameter
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

// ---- Encoder pins (front-left only) -- see 4-encoder pin plan in header ----
// D18 = TX1 on the board label -- usable as digital I/O when Serial1 is not
// in use, which is the case here. D19 = RX1, also free and also
// interrupt-capable, matching the physical FL wiring in encoder_test.ino.
const int ENC_A_PIN = 18;  // interrupt-capable, connect to encoder channel A
const int ENC_B_PIN = 19;  // direction sense,   connect to encoder channel B

volatile long encoderCount = 0;

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

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEncoderA, RISING);

  stopAll();
  Serial.println(F("main_robot ready. Commands: F<cm>  S  G"));
}

void loop() {
  handleSerialCommands();

  if (state == DRIVING) {
    if (abs(encoderCount) >= targetCounts) {
      stopAll();
      state = IDLE;
      Serial.println(F("Target reached, stopped."));
    }
    // NOTE: open-loop speed with closed-loop distance cutoff -- does not yet
    // correct for left/right drift. Track per-wheel encoders independently
    // and adjust individual PWM if drift becomes a real problem in testing.
  }
}

void handleSerialCommands() {
  if (Serial.available() <= 0) return;

  char cmd = Serial.read();

  if (cmd == 'F') {
    float distanceCm = Serial.parseFloat();
    if (distanceCm > 0) {
      noInterrupts();
      encoderCount = 0;
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

void handleEncoderA() {
  if (digitalRead(ENC_B_PIN) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
