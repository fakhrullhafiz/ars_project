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
// TEMPORARY: lowered from 180 to 90 for visual direction-check testing only —
// slow enough to watch each wheel/gear turn and confirm it matches the
// assigned rotation. Lower duty cycle is strictly safer for the motors than
// 180, not riskier. Restore to MAX_PWM = 180 once direction is confirmed;
// don't leave this in place for real driving or thermal testing.
const int MAX_PWM = 80;   // out of 255 (~35%) — TEMP for direction-check, revert to 180 after
const int MIN_PWM = 60;   // below this the motor may not overcome static friction

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
