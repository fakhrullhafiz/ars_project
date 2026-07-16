/*
  motor_control.ino
  ----------------------------------------------------------------------------
  Layer 0 baseline: drives all 4 mecanum wheels via two MC33886VW dual H-bridge
  driver modules, controlled from an Arduino Mega 2560.

  WIRING — all 8 control pins are on the right-side header (~D2 through ~D9):
    Driver 1 (Front motors)           Driver 2 (Rear motors)
      IN1 -> pin 2  (Front-Left  A)     IN1 -> pin 6  (Rear-Left  A)
      IN2 -> pin 3  (Front-Left  B)     IN2 -> pin 7  (Rear-Left  B)
      IN3 -> pin 4  (Front-Right A)     IN3 -> pin 8  (Rear-Right A)
      IN4 -> pin 5  (Front-Right B)     IN4 -> pin 9  (Rear-Right B)
  All 8 pins above are PWM-capable (marked ~ on the board silkscreen).
  These boards do not expose a separate PWM/enable pin — speed is controlled
  by applying analogWrite() directly to the active IN pin while holding the
  other IN pin LOW. See setMotor() below.
  Mega 5V  -> both driver 5V+ (logic supply)
  Mega GND -> both driver GND (logic ground)
  Motor battery + -> driver motor power input (screw terminal)
  Motor battery - -> driver motor GND (screw terminal) — tie to Mega GND too

  *** PWM CEILING — READ BEFORE CHANGING ***
  The JGB37-520 motors are rated 6.0V DC. The battery is a confirmed 2S LiPo
  (7.4V nominal / 8.4V full charge), which exceeds that rating. This driver
  passes battery voltage straight through to the motors — no onboard
  regulation. Running at 100% PWM drives the motors at full battery voltage,
  well above their 6V rating.
  MAX_PWM below caps duty cycle as the mitigation: 180/255 ≈ 70% matches
  6.0V / 8.4V ≈ 71%, so full duty cycle works out close to the motors' rated
  voltage. Confirm motor casing stays cool (not hot) during a multi-minute
  test run before trusting this value — start any first power-on test at
  brief, short presses rather than sustained driving.
  See CLAUDE.md for the full reasoning.
*/

// ---- Tunable safety ceiling — see comment block above before changing ----
const int MAX_PWM = 180;  // out of 255 (~70%) — matches confirmed 2S battery (6.0V / 8.4V)
const int MIN_PWM = 60;   // below this the motor may not overcome static friction

// ---- Diagnostic/bring-up only: slow single-motor test speed ----
// Deliberately low so the shaft turns slowly enough to read a sticky-note flag
// and confirm which corner + which direction. Well under MAX_PWM. With no wheel
// load the motor spins faster for a given duty, so this stays gentle. Bump it a
// little if a motor won't start turning; drop it if it's still too fast to read.
const int TEST_PWM = 90;  // out of 255 — bring-up identification only, not driving

// ---- All pins on the right-side header, PWM-capable (marked ~ on board) ----
const int FL_IN1 = 2,  FL_IN2 = 3;   // Front-Left  (~D2, ~D3)
const int FR_IN1 = 4,  FR_IN2 = 5;   // Front-Right (~D4, ~D5)
const int RL_IN1 = 6,  RL_IN2 = 7;   // Rear-Left   (~D6, ~D7)
const int RR_IN1 = 8,  RR_IN2 = 9;   // Rear-Right  (~D8, ~D9)

void setup() {
  Serial.begin(115200);

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

  stopAll();
  Serial.println(F("motor_control ready. Type a single character + Enter:"));
  Serial.println(F("  w=forward  s=reverse  a=strafe-left  d=strafe-right"));
  Serial.println(F("  q=rotate-left  e=rotate-right  x=stop"));
  Serial.println(F("--- single-motor ID test (slow) ---"));
  Serial.println(F("  1=FL fwd  2=FR fwd  3=RL fwd  4=RR fwd"));
  Serial.println(F("  5=FL rev  6=FR rev  7=RL rev  8=RR rev  (x=stop)"));
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'w': driveForward();  Serial.println(F("forward"));      break;
      case 's': driveReverse();  Serial.println(F("reverse"));      break;
      case 'a': strafeLeft();    Serial.println(F("strafe left"));  break;
      case 'd': strafeRight();   Serial.println(F("strafe right")); break;
      case 'q': rotateLeft();    Serial.println(F("rotate left"));  break;
      case 'e': rotateRight();   Serial.println(F("rotate right")); break;
      case 'x': stopAll();       Serial.println(F("stop"));         break;

      // ---- single-motor ID test (slow) — one corner at a time ----
      case '1': stopAll(); setMotor(FL_IN1, FL_IN2,  TEST_PWM); Serial.println(F("FL forward (slow)")); break;
      case '2': stopAll(); setMotor(FR_IN1, FR_IN2,  TEST_PWM); Serial.println(F("FR forward (slow)")); break;
      case '3': stopAll(); setMotor(RL_IN1, RL_IN2,  TEST_PWM); Serial.println(F("RL forward (slow)")); break;
      case '4': stopAll(); setMotor(RR_IN1, RR_IN2,  TEST_PWM); Serial.println(F("RR forward (slow)")); break;
      case '5': stopAll(); setMotor(FL_IN1, FL_IN2, -TEST_PWM); Serial.println(F("FL reverse (slow)")); break;
      case '6': stopAll(); setMotor(FR_IN1, FR_IN2, -TEST_PWM); Serial.println(F("FR reverse (slow)")); break;
      case '7': stopAll(); setMotor(RL_IN1, RL_IN2, -TEST_PWM); Serial.println(F("RL reverse (slow)")); break;
      case '8': stopAll(); setMotor(RR_IN1, RR_IN2, -TEST_PWM); Serial.println(F("RR reverse (slow)")); break;
      default: break;
    }
  }
}

// ----------------------------------------------------------------------------
// Mecanum kinematics:
//   Forward:      all 4 wheels spin forward
//   Strafe right: FL fwd, FR rev, RL rev, RR fwd
//   Rotate left:  left wheels rev, right wheels fwd
//
// setMotor(): speed is -255..255. Positive = forward, negative = reverse.
// The active IN pin gets analogWrite(speed) for PWM speed control;
// the other IN pin is held LOW. Both LOW = motor coast/stop.

void setMotor(int in1, int in2, int speed) {
  int clamped = constrain(abs(speed), 0, MAX_PWM);
  if (clamped > 0 && clamped < MIN_PWM) clamped = MIN_PWM;
  if (speed > 0) { analogWrite(in1, clamped); digitalWrite(in2, LOW); }
  else if (speed < 0) { digitalWrite(in1, LOW); analogWrite(in2, clamped); }
  else { digitalWrite(in1, LOW); digitalWrite(in2, LOW); }
}

void driveForward() {
  setMotor(FL_IN1, FL_IN2,  MAX_PWM);
  setMotor(FR_IN1, FR_IN2,  MAX_PWM);
  setMotor(RL_IN1, RL_IN2,  MAX_PWM);
  setMotor(RR_IN1, RR_IN2,  MAX_PWM);
}

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

void stopAll() {
  setMotor(FL_IN1, FL_IN2, 0);
  setMotor(FR_IN1, FR_IN2, 0);
  setMotor(RL_IN1, RL_IN2, 0);
  setMotor(RR_IN1, RR_IN2, 0);
}
