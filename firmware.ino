/*
 * Vision-Guided Prosthetic Hand - Arduino firmware
 * Board : Arduino Uno / Nano
 * Servos: 5x MG995 (thumb, index, middle, ring, pinky)
 *
 * WIRING / POWER (important):
 *   - Servo signal pins -> D3, D5, D6, D9, D10
 *   - MG995 servos draw up to ~1A each under stall. Power them from an
 *     EXTERNAL 5-6V supply (>= 5A). NEVER from the Arduino 5V pin.
 *   - Connect external supply GND to Arduino GND (common ground).
 *   - Add a 1000uF capacitor across the servo supply.
 *
 * SERIAL PROTOCOL (115200 baud, newline-terminated ASCII):
 *   O              -> open hand
 *   C              -> close hand (full fist)
 *   G1             -> power grip   (all fingers wrap)
 *   G2             -> pinch grip   (thumb + index)
 *   G3             -> tripod grip  (thumb + index + middle)
 *   P a b c d e    -> custom angles 0-180 for each finger
 *   ?              -> report current angles
 * Replies: "OK <cmd>" or "ERR <reason>"
 */

#include <Servo.h>

const uint8_t NUM_FINGERS = 5;
const uint8_t SERVO_PINS[NUM_FINGERS] = {3, 5, 6, 9, 10};  // thumb..pinky

// Calibrate these per finger so the tendon/linkage doesn't over-stretch.
// Angle at fully OPEN and fully CLOSED for each finger.
// Swap values for a finger if its servo is mounted in reverse.
const int OPEN_ANGLE[NUM_FINGERS]   = {  0,   0,   0,   0,   0};
const int CLOSED_ANGLE[NUM_FINGERS] = {150, 160, 160, 160, 160};

// Grip presets (angles per finger: thumb, index, middle, ring, pinky)
const int GRIP_POWER[NUM_FINGERS]  = {150, 160, 160, 160, 160};
const int GRIP_PINCH[NUM_FINGERS]  = {120, 110,   0,   0,   0};
const int GRIP_TRIPOD[NUM_FINGERS] = {120, 110, 110,   0,   0};

const unsigned long STEP_INTERVAL_MS = 15;  // smoothing update period
const int STEP_DEG = 3;                     // degrees per update

Servo fingers[NUM_FINGERS];
int currentAngle[NUM_FINGERS];
int targetAngle[NUM_FINGERS];

String rxBuffer;
unsigned long lastStep = 0;

int clampAngle(int i, int a) {
  int lo = min(OPEN_ANGLE[i], CLOSED_ANGLE[i]);
  int hi = max(OPEN_ANGLE[i], CLOSED_ANGLE[i]);
  return constrain(a, lo, hi);
}

void setTargets(const int *angles) {
  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    targetAngle[i] = clampAngle(i, angles[i]);
  }
}

void reportAngles() {
  Serial.print(F("ANG"));
  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    Serial.print(' ');
    Serial.print(currentAngle[i]);
  }
  Serial.println();
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "O") {
    setTargets(OPEN_ANGLE);
    Serial.println(F("OK O"));
  } else if (cmd == "C") {
    setTargets(CLOSED_ANGLE);
    Serial.println(F("OK C"));
  } else if (cmd == "G1") {
    setTargets(GRIP_POWER);
    Serial.println(F("OK G1"));
  } else if (cmd == "G2") {
    setTargets(GRIP_PINCH);
    Serial.println(F("OK G2"));
  } else if (cmd == "G3") {
    setTargets(GRIP_TRIPOD);
    Serial.println(F("OK G3"));
  } else if (cmd == "?") {
    reportAngles();
  } else if (cmd.charAt(0) == 'P') {
    int vals[NUM_FINGERS];
    int count = 0;
    int idx = 1;
    while (count < NUM_FINGERS && idx < (int)cmd.length()) {
      while (idx < (int)cmd.length() && cmd.charAt(idx) == ' ') idx++;
      if (idx >= (int)cmd.length()) break;
      int next = cmd.indexOf(' ', idx);
      if (next < 0) next = cmd.length();
      vals[count++] = cmd.substring(idx, next).toInt();
      idx = next;
    }
    if (count == NUM_FINGERS) {
      setTargets(vals);
      Serial.println(F("OK P"));
    } else {
      Serial.println(F("ERR need 5 angles"));
    }
  } else {
    Serial.println(F("ERR unknown"));
  }
}

void setup() {
  Serial.begin(115200);
  rxBuffer.reserve(48);

  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    currentAngle[i] = OPEN_ANGLE[i];
    targetAngle[i]  = OPEN_ANGLE[i];
    fingers[i].attach(SERVO_PINS[i], 500, 2400);  // MG995 pulse range (us)
    fingers[i].write(currentAngle[i]);
  }
  Serial.println(F("READY"));
}

void loop() {
  // --- Read serial commands ---
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleCommand(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      if (rxBuffer.length() < 40) rxBuffer += c;
    }
  }

  // --- Smoothly move servos toward targets (avoids current spikes) ---
  unsigned long now = millis();
  if (now - lastStep >= STEP_INTERVAL_MS) {
    lastStep = now;
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
      if (currentAngle[i] != targetAngle[i]) {
        int diff = targetAngle[i] - currentAngle[i];
        if (abs(diff) <= STEP_DEG) currentAngle[i] = targetAngle[i];
        else currentAngle[i] += (diff > 0) ? STEP_DEG : -STEP_DEG;
        fingers[i].write(currentAngle[i]);
      }
    }
  }
}
