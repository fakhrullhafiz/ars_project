/*
  motor_control.ino
  ----------------------------------------------------------------------------
  Layer 0 baseline: drives all 4 mecanum wheels via two MC33886VW dual H-bridge
  driver modules, controlled from an Arduino Mega 2560.

  WIRING (confirm against your actual board before running):
    Driver 1 (Front motors)            Driver 2 (Rear motors)
      IN1 -> pin 22  (Front-Left dir A)   IN1 -> pin 30 (Rear-Left dir A)
      IN2 -> pin 23  (Front-Left dir B)   IN2 -> pin 31 (Rear-Left dir B)
      IN3 -> pin 24  (Front-Right dir A)  IN3 -> pin 32 (Rear-Right dir A)
      IN4 -> pin 25  (Front-Right dir B)  IN4 -> pin 33 (Rear-Right dir B)
      PWM1 -> pin 2  (Front-Left speed)   PWM1 -> pin 6 (Rear-Left speed)
      PWM2 -> pin 3  (Front-Right speed)  PWM2 -> pin 7 (Rear-Right speed)

  Pins above are PLACEHOLDERS based on typical Mega PWM-capable pins (2,3,5,6,7,8,9...).
  Re-check against the Arduino Mega 2560 pinout (Section 4.3 of the component doc)
  before wiring — any of the 15 hardware PWM pins will work for the PWM lines,
  any free digital pin works for the directional IN lines.

  *** PWM CEILING — READ BEFORE CHANGING ***
  The JGB37-520 motors are rated 6.0V DC. The battery pack (3S/11.1V or 4S/14.8V)
  significantly exceeds that. This driver passes battery voltage straight through
  to the motor — there is no voltage regulation on the motor power path. Running
  at 100% PWM duty cycle overdrives the motors well beyond spec.
  MAX_PWM below caps duty cycle as the mitigation. Increase only after confirming
  motor casing stays cool (not hot to the touch) during a multi-minute test run.
  See CLAUDE.md / project plan Section 2.2 for the full reasoning.
*/

// ---- Tunable safety ceiling — see comment block above before changing ----
const int MAX_PWM = 120;     // out of 255 (~47%) — starting conservative value, tune up carefully
const int MIN_PWM = 60;      // minimum PWM to actually overcome static friction; below this, wheels may not turn at all

// ---- Front-Left motor (Driver 1, Channel 1) ----
const int FL_IN1 = 22;
const int FL_IN2 = 23;
const int FL_PWM = 2;

// ---- Front-Right motor (Driver 1, Channel 2) ----
const int FR_IN1 = 24;
const int FR_IN2 = 25;
const int FR_PWM = 3;

// ---- Rear-Left motor (Driver 2, Channel 1) ----
const int RL_IN1 = 30;
const int RL_IN2 = 31;
const int RL_PWM = 6;

// ---- Rear-Right motor (Driver 2, Channel 2) ----
const int RR_IN1 = 32;
const int RR_IN2 = 33;
const int RR_PWM = 7;

void setup() {
  Serial.begin(115200);

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_PWM, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_PWM, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT); pinMode(RL_PWM, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_PWM, OUTPUT);

  stopAll();
  Serial.println(F("motor_control ready. Type a single character + Enter:"));
  Serial.println(F("  w=forward  s=reverse  a=strafe-left  d=strafe-right"));
  Serial.println(F("  q=rotate-left  e=rotate-right  x=stop"));
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'w': driveForward();    Serial.println(F("forward"));     break;
      case 's': driveReverse();    Serial.println(F("reverse"));     break;
      case 'a': strafeLeft();      Serial.println(F("strafe left")); break;
      case 'd': strafeRight();     Serial.println(F("strafe right"));break;
      case 'q': rotateLeft();      Serial.println(F("rotate left")); break;
      case 'e': rotateRight();     Serial.println(F("rotate right"));break;
      case 'x': stopAll();         Serial.println(F("stop"));        break;
      default: break; // ignore newline / unrecognized chars
    }
  }
}

// ----------------------------------------------------------------------------
// Mecanum kinematics (see component doc Section 10.2):
//   Forward:        all 4 wheels forward
//   Strafe right:   FL fwd, FR rev, RL rev, RR fwd
//   Rotate in place: left wheels fwd, right wheels rev (or vice versa)
// Each motor has an independent direction (set via IN1/IN2 pair) and speed
// (via PWM pin). setMotor() below handles both at once for one wheel.
// ----------------------------------------------------------------------------

void setMotor(int in1, int in2, int pwmPin, int speed) {
  // speed: -255..255. Sign sets direction, magnitude sets PWM (clamped to MAX_PWM).
  int clamped = constrain(abs(speed), 0, MAX_PWM);
  if (clamped > 0 && clamped < MIN_PWM) clamped = MIN_PWM; // avoid stall-without-motion zone

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
  setMotor(FL_IN1, FL_IN2, FL_PWM,  MAX_PWM);
  setMotor(FR_IN1, FR_IN2, FR_PWM,  MAX_PWM);
  setMotor(RL_IN1, RL_IN2, RL_PWM,  MAX_PWM);
  setMotor(RR_IN1, RR_IN2, RR_PWM,  MAX_PWM);
}

void driveReverse() {
  setMotor(FL_IN1, FL_IN2, FL_PWM, -MAX_PWM);
  setMotor(FR_IN1, FR_IN2, FR_PWM, -MAX_PWM);
  setMotor(RL_IN1, RL_IN2, RL_PWM, -MAX_PWM);
  setMotor(RR_IN1, RR_IN2, RR_PWM, -MAX_PWM);
}

void strafeLeft() {
  setMotor(FL_IN1, FL_IN2, FL_PWM, -MAX_PWM);
  setMotor(FR_IN1, FR_IN2, FR_PWM,  MAX_PWM);
  setMotor(RL_IN1, RL_IN2, RL_PWM,  MAX_PWM);
  setMotor(RR_IN1, RR_IN2, RR_PWM, -MAX_PWM);
}

void strafeRight() {
  setMotor(FL_IN1, FL_IN2, FL_PWM,  MAX_PWM);
  setMotor(FR_IN1, FR_IN2, FR_PWM, -MAX_PWM);
  setMotor(RL_IN1, RL_IN2, RL_PWM, -MAX_PWM);
  setMotor(RR_IN1, RR_IN2, RR_PWM,  MAX_PWM);
}

void rotateLeft() {
  setMotor(FL_IN1, FL_IN2, FL_PWM, -MAX_PWM);
  setMotor(FR_IN1, FR_IN2, FR_PWM,  MAX_PWM);
  setMotor(RL_IN1, RL_IN2, RL_PWM, -MAX_PWM);
  setMotor(RR_IN1, RR_IN2, RR_PWM,  MAX_PWM);
}

void rotateRight() {
  setMotor(FL_IN1, FL_IN2, FL_PWM,  MAX_PWM);
  setMotor(FR_IN1, FR_IN2, FR_PWM, -MAX_PWM);
  setMotor(RL_IN1, RL_IN2, RL_PWM,  MAX_PWM);
  setMotor(RR_IN1, RR_IN2, RR_PWM, -MAX_PWM);
}

void stopAll() {
  setMotor(FL_IN1, FL_IN2, FL_PWM, 0);
  setMotor(FR_IN1, FR_IN2, FR_PWM, 0);
  setMotor(RL_IN1, RL_IN2, RL_PWM, 0);
  setMotor(RR_IN1, RR_IN2, RR_PWM, 0);
}
