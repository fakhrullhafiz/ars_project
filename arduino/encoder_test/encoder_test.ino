/*
  encoder_test.ino
  ----------------------------------------------------------------------------
  Bring-up and calibration sketch for the JGB37-520 magnetic Hall-effect
  quadrature encoders (component doc Section 7). Run this BEFORE attempting
  closed-loop "drive to distance" in main_robot.ino (Layer 1).

  PURPOSE:
    1. Confirm each encoder is wired correctly and produces clean pulse counts.
    2. Measure counts-per-revolution and, combined with wheel diameter,
       derive counts-per-cm — the calibration constant Layer 1 code needs.

  WIRING (per encoder, component doc Section 7.3):
    Enc A -> any interrupt-capable pin (Mega: 2, 3, 18, 19, 20, 21)
    Enc B -> any digital pin (used for direction sensing)
    VCC   -> +5V (encoder logic supply, separate from motor power)
    GND   -> common ground

  This sketch wires up ONE encoder (front-left) as a worked example.
  Duplicate the pattern for the other 3 wheels once this one is confirmed
  working — don't copy-paste all 4 untested at once, debug one at a time.

  CALIBRATION PROCEDURE:
    1. Upload this sketch, open Serial Monitor at 115200 baud.
    2. Manually rotate the wheel exactly 1 full revolution by hand, slowly.
    3. Read the printed count — that's counts-per-revolution for this encoder.
    4. Measure wheel diameter (mm), compute wheel circumference = pi * diameter.
    5. counts_per_cm = counts_per_revolution / (circumference_mm / 10)
    6. Record this value — main_robot.ino needs it for distance-based driving.
*/

const int ENC_A_PIN = 2;   // interrupt-capable pin
const int ENC_B_PIN = 4;   // direction sense pin

volatile long encoderCount = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  // Trigger on every rising edge of Encoder A; read B to determine direction.
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEncoderA, RISING);

  Serial.println(F("encoder_test ready."));
  Serial.println(F("Rotate the wheel by hand and watch the count below."));
  Serial.println(F("Type 'r' + Enter to reset the count to zero."));
}

void loop() {
  static long lastPrinted = -1;
  if (encoderCount != lastPrinted) {
    Serial.print(F("Count: "));
    Serial.println(encoderCount);
    lastPrinted = encoderCount;
  }

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r') {
      noInterrupts();
      encoderCount = 0;
      interrupts();
      Serial.println(F("-- count reset to 0 --"));
    }
  }
}

void handleEncoderA() {
  // Quadrature direction logic: if B is HIGH when A rises, one direction;
  // if B is LOW, the other. Adjust the comparison if direction reads backwards
  // on your actual hardware (swapping A/B wiring also fixes this physically).
  if (digitalRead(ENC_B_PIN) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
