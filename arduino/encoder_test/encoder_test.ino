/*
  encoder_test.ino
  ----------------------------------------------------------------------------
  Bring-up and calibration sketch for all 4 JGB37-520 magnetic Hall-effect
  quadrature encoders (component doc Section 7). Run this BEFORE attempting
  closed-loop "drive to distance" in main_robot.ino (Layer 1).

  PURPOSE:
    1. Confirm each encoder is wired correctly and produces clean pulse counts.
    2. Measure counts-per-revolution and, combined with wheel diameter,
       derive counts-per-cm -- the calibration constant Layer 1 code needs.

  WIRING (matches physical wiring done 2026-07-16 -- see CLAUDE.md):
    FL: phase A -> D18 (hw interrupt), phase B -> D19
    FR: phase A -> D20 (hw interrupt), phase B -> D21
    RL: phase A -> D11 (pin-change interrupt), phase B -> D24
    RR: phase A -> D12 (pin-change interrupt), phase B -> D25
    All 4: encoder VCC -> +5V (NOT the lever-nut battery block, that's raw
    battery voltage and will fry the encoder's Hall-sensor electronics),
    GND -> common ground (shared with Mega GND / driver GND / battery- rail).

  Only D2, D3, D18, D19, D20, D21 are true hardware-interrupt pins on the
  Mega, and D2/D3 are already taken by the FL motor. FL/FR encoders use
  attachInterrupt() as before; RL/RR share the Mega's PCINT0 group
  (D10-D13, D50-D53) via raw AVR registers instead -- not a library, to
  avoid adding a dependency without asking first.

  CALIBRATION PROCEDURE:
    1. Upload this sketch, open Serial Monitor at 115200 baud.
    2. Rotate ONE wheel at a time, exactly 1 full revolution, by hand, slowly.
    3. Read that wheel's printed count -- that's counts-per-revolution for it.
    4. Measure wheel diameter (mm), compute wheel circumference = pi * diameter.
    5. counts_per_cm = counts_per_revolution / (circumference_mm / 10)
    6. Record each wheel's value. main_robot.ino currently only consumes the
       front-left value (COUNTS_PER_CM) -- see that file's header comment.
    7. Type 'r' + Enter to reset all 4 counts to zero between wheels.
*/

const int FL_A = 18, FL_B = 19;
const int FR_A = 20, FR_B = 21;
const int RL_A = 11, RL_B = 24;  // RL_A = Mega pin 11 = AVR PB5 = PCINT5
const int RR_A = 12, RR_B = 25;  // RR_A = Mega pin 12 = AVR PB6 = PCINT6

volatile long flCount = 0, frCount = 0, rlCount = 0, rrCount = 0;
volatile uint8_t lastPINB = 0;

void setup() {
  Serial.begin(115200);

  pinMode(FL_A, INPUT_PULLUP); pinMode(FL_B, INPUT_PULLUP);
  pinMode(FR_A, INPUT_PULLUP); pinMode(FR_B, INPUT_PULLUP);
  pinMode(RL_A, INPUT_PULLUP); pinMode(RL_B, INPUT_PULLUP);
  pinMode(RR_A, INPUT_PULLUP); pinMode(RR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_A), handleFL, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_A), handleFR, RISING);

  // RL/RR: pin-change interrupt on PCINT0 group (covers D10-D13, D50-D53).
  lastPINB = PINB;
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT5) | (1 << PCINT6);

  Serial.println(F("encoder_test ready -- all 4 wheels."));
  Serial.println(F("Rotate ONE wheel by hand at a time and watch its count."));
  Serial.println(F("Type 'r' + Enter to reset all counts to zero."));
}

void loop() {
  static long lastFL = -1, lastFR = -1, lastRL = -1, lastRR = -1;
  long fl, fr, rl, rr;

  noInterrupts();
  fl = flCount; fr = frCount; rl = rlCount; rr = rrCount;
  interrupts();

  if (fl != lastFL || fr != lastFR || rl != lastRL || rr != lastRR) {
    Serial.print(F("FL:")); Serial.print(fl);
    Serial.print(F(" FR:")); Serial.print(fr);
    Serial.print(F(" RL:")); Serial.print(rl);
    Serial.print(F(" RR:")); Serial.println(rr);
    lastFL = fl; lastFR = fr; lastRL = rl; lastRR = rr;
  }

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r') {
      noInterrupts();
      flCount = 0; frCount = 0; rlCount = 0; rrCount = 0;
      interrupts();
      Serial.println(F("-- all counts reset to 0 --"));
    }
  }
}

// Quadrature direction logic: if B is HIGH when A rises, one direction;
// if B is LOW, the other. Swapping A/B wiring on a wheel flips this physically.
void handleFL() { if (digitalRead(FL_B) == HIGH) flCount++; else flCount--; }
void handleFR() { if (digitalRead(FR_B) == HIGH) frCount++; else frCount--; }

// Fires on ANY change on PB0-PB7 (D10-D13, D50-D53) -- must check which of
// our two watched pins actually changed, and that it was a rising edge, to
// match the RISING-only behavior used for FL/FR above.
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
