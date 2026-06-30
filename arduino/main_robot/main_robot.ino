/*
  main_robot.ino
  ----------------------------------------------------------------------------
  Layer 1 (closed-loop "drive to distance") with a Layer 2 extension point for
  obstacle-stop commands from the RPLIDAR/perception side.

  PREREQUISITES — confirm both of these work BEFORE using this sketch:
    - motor_control.ino: all 4 wheels respond correctly to direction/PWM
    - encoder_test.ino:  you have a calibrated COUNTS_PER_CM value (see below)

  This combines motor driving + encoder feedback so the robot can be told
  "drive forward 100 cm" and have it actually stop at ~100 cm, rather than
  guessing with a fixed time delay.

  *** Update COUNTS_PER_CM below with YOUR calibrated value from encoder_test.ino
  before relying on this for distance accuracy. The placeholder value here is
  a rough guess and will NOT be accurate for your actual wheels. ***

  SERIAL COMMAND PROTOCOL (interface for Layer 2 / perception side):
    Over USB serial (115200 baud), this sketch accepts single-character commands:
      'F' <distance_cm>  -- drive forward N cm then stop  (e.g. "F100\n")
      'S'                -- emergency stop, immediately zero all motors
      'G'                -- resume/go (clears a stop triggered by 'S')
    This is intentionally simple (not a binary protocol) so it's easy for
    Person C's RPLIDAR-side code to send plain text commands over serial
    without needing a shared binary format. Extend this protocol together
    once Layer 2 obstacle-detection logic exists — don't let one side change
    it unilaterally, since both sides depend on the exact command characters.
*/

// ---- Calibration — REPLACE with your measured value from encoder_test.ino ----
const float COUNTS_PER_CM = 20.0;  // PLACEHOLDER — recalibrate, see file header

// ---- Safety PWM ceiling — same reasoning as motor_control.ino, keep in sync ----
const int MAX_PWM = 120;
const int MIN_PWM = 60;

// ---- Motor pins (must match your actual wiring — see motor_control.ino) ----
const int FL_IN1 = 22, FL_IN2 = 23, FL_PWM = 2;
const int FR_IN1 = 24, FR_IN2 = 25, FR_PWM = 3;
const int RL_IN1 = 30, RL_IN2 = 31, RL_PWM = 6;
const int RR_IN1 = 32, RR_IN2 = 33, RR_PWM = 7;

// ---- Encoder pins — front-left shown; extend to all 4 wheels once this works ----
const int ENC_A_PIN = 18;  // separate interrupt pin from anything in encoder_test.ino bring-up
const int ENC_B_PIN = 19;

volatile long encoderCount = 0;

// ---- State machine ----
enum RobotState { IDLE, DRIVING, EMERGENCY_STOPPED };
RobotState state = IDLE;
long targetCounts = 0;

void setup() {
  Serial.begin(115200);

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_PWM, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_PWM, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT); pinMode(RL_PWM, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_PWM, OUTPUT);

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
    // NOTE: this drives open-loop speed with a closed-loop distance cutoff —
    // it does not yet correct for left/right drift (e.g. one side reaching
    // target counts before the other due to wheel slip or motor variance).
    // A straightforward next improvement: track each wheel's encoder
    // independently and adjust individual wheel PWM to keep them in sync.
    // Left as a deliberate v1 simplification — flag if drift becomes a problem
    // during Week 3 testing.
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
    // Emergency stop — this is the hook Layer 2 (RPLIDAR obstacle detection)
    // calls into. Person C: send the single character 'S' over serial the
    // moment an obstacle is detected within the stop threshold.
    stopAll();
    state = EMERGENCY_STOPPED;
    Serial.println(F("EMERGENCY STOP"));
  } else if (cmd == 'G') {
    // Resume after an emergency stop has been cleared (obstacle no longer
    // in path). Does NOT automatically resume the previous drive command —
    // intentional: re-evaluating whether it's still safe to continue belongs
    // on the perception side, not assumed here.
    if (state == EMERGENCY_STOPPED) {
      state = IDLE;
      Serial.println(F("Cleared emergency stop, ready for new command."));
    }
  }
}

void setMotor(int in1, int in2, int pwmPin, int speed) {
  int clamped = constrain(abs(speed), 0, MAX_PWM);
  if (clamped > 0 && clamped < MIN_PWM) clamped = MIN_PWM;

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    clamped = 0;
  }
  analogWrite(pwmPin, clamped);
}

void driveForward() {
  setMotor(FL_IN1, FL_IN2, FL_PWM, MAX_PWM);
  setMotor(FR_IN1, FR_IN2, FR_PWM, MAX_PWM);
  setMotor(RL_IN1, RL_IN2, RL_PWM, MAX_PWM);
  setMotor(RR_IN1, RR_IN2, RR_PWM, MAX_PWM);
}

void stopAll() {
  setMotor(FL_IN1, FL_IN2, FL_PWM, 0);
  setMotor(FR_IN1, FR_IN2, FR_PWM, 0);
  setMotor(RL_IN1, RL_IN2, RL_PWM, 0);
  setMotor(RR_IN1, RR_IN2, RR_PWM, 0);
}

void handleEncoderA() {
  if (digitalRead(ENC_B_PIN) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
