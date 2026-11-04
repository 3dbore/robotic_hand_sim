/*
 * ELIOS 6-axis slave — ESP32 + PCA9685
 * ------------------------------------
 * Drop-in replacement for slave_code/6-axis/6-axis.ino (Arduino Uno).
 * Same serial protocol, so arm-hil-toolkit.py works unchanged:
 *
 *   "F0,F1,F2,F3,F4,F5\n"   integer degrees 0..180, empty field = release servo
 *    F0 Gripper | F1 Wrist Yaw | F2 Wrist Pitch | F3 Arm B | F4 Arm A (mirrored pair) | F5 Base
 *
 * Wiring
 *   Base (azimuth)  -> ESP32 GPIO BASE_PIN (direct)
 *   Everything else -> PCA9685 over I2C (SDA 21, SCL 22)
 *
 *   PCA9685 numbering below uses YOUR labels 1..16 (board channel = label - 1):
 *     1  Arm A (primary)
 *     2  Arm A (mirror, faces motor 1 -> written as 180 - angle)
 *     3  Arm B
 *     5  Wrist Pitch
 *     11 Wrist Yaw
 *     16 Gripper
 *
 * Libraries (Arduino Library Manager):
 *   - "ESP32Servo" by Kevin Harrington / madhephaestus
 *   - "Adafruit PWM Servo Driver Library" by Adafruit
 */

#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ================= CONFIGURATION =================
const long SERIAL_BAUD = 9600;          // must match SERIAL_BAUD in hardware_mapping.py

// --- Direct-wired base servo ---
const int BASE_PIN = 13;                // change to the GPIO you actually used

// --- PCA9685 ---
const uint8_t PCA_ADDR = 0x40;
const int I2C_SDA = 21;
const int I2C_SCL = 22;
const uint32_t PCA_OSC_HZ = 27000000;   // trim if pulses measure off (typ. 25–27 MHz)

// Logical axes, in serial field order (DO NOT reorder — the master depends on it)
enum Axis { GRIPPER = 0, WRIST_YAW, WRIST_PITCH, ARM_B, ARM_A, BASE, AXIS_COUNT };

// PCA9685 label (1..16) per axis; 0 = not on the PCA (direct GPIO)
const uint8_t PCA_LABEL[AXIS_COUNT] = {
  16,   // GRIPPER
  11,   // WRIST_YAW
  5,    // WRIST_PITCH
  3,    // ARM_B
  1,    // ARM_A (primary)
  0     // BASE  -> BASE_PIN
};
const uint8_t ARM_A_MIRROR_LABEL = 2;   // opposing motor of Arm A

// Small offset to remove fighting between the two Arm A motors if their
// horns aren't perfectly aligned. Tune with power on until neither buzzes.
const float ARM_A_MIRROR_TRIM_DEG = 0.0f;

// MG996R pulse range
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Speed limit (same as Uno firmware)
const float MAX_SPEED_DEG_PER_SEC = 60.0f;

// true  = empty field releases the servo (no PWM pulse)
// false = empty field keeps the servo holding its last position
const bool EMPTY_MEANS_NO_SIGNAL = true;

const unsigned long LINE_TIMEOUT_MS = 30;
const int MAX_LINE_LENGTH = 64;
// =================================================

Adafruit_PWMServoDriver pca(PCA_ADDR);
Servo baseServo;

bool attached[AXIS_COUNT] = {false};
float currentAngle[AXIS_COUNT];
float targetAngle[AXIS_COUNT];
unsigned long lastMoveTime[AXIS_COUNT] = {0};

String inputLine;
unsigned long lastCharTime = 0;

// ---------- low-level output ----------

static inline uint8_t pcaChannel(uint8_t label) { return label - 1; }

static inline uint16_t angleToUs(float angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return (uint16_t)lroundf(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180.0f));
}

void pcaOff(uint8_t label) {
  pca.setPWM(pcaChannel(label), 0, 4096);   // full-off bit: line held LOW, no pulses
}

// Writes a (fractional) angle to the physical motor(s) behind one axis.
// PCA9685 has ~4.9 us resolution, so sub-degree motion is actually smoother than the Uno.
void writeAxis(int axis, float angle) {
  if (axis == BASE) {
    baseServo.writeMicroseconds(angleToUs(angle));
    return;
  }
  pca.writeMicroseconds(pcaChannel(PCA_LABEL[axis]), angleToUs(angle));

  if (axis == ARM_A) {
    float mirrored = 180.0f - angle + ARM_A_MIRROR_TRIM_DEG;
    pca.writeMicroseconds(pcaChannel(ARM_A_MIRROR_LABEL), angleToUs(mirrored));
  }
}

// ---------- attach / release ----------

void moveServo(int axis, int angle) {
  if (!attached[axis]) {
    if (axis == BASE) {
      baseServo.setPeriodHertz(50);
      baseServo.attach(BASE_PIN, SERVO_MIN_US, SERVO_MAX_US);
    }
    attached[axis] = true;
    // Like the Uno firmware: first command after release jumps straight there
    // (we don't know where the unpowered servo was left).
    currentAngle[axis] = angle;
    writeAxis(axis, angle);
    lastMoveTime[axis] = millis();
  }
  targetAngle[axis] = angle;
}

void releaseServo(int axis) {
  if (!attached[axis]) return;

  if (axis == BASE) {
    baseServo.detach();
    pinMode(BASE_PIN, OUTPUT);
    digitalWrite(BASE_PIN, LOW);
  } else {
    pcaOff(PCA_LABEL[axis]);
    if (axis == ARM_A) pcaOff(ARM_A_MIRROR_LABEL);
  }

  attached[axis] = false;
  currentAngle[axis] = 90.0f;
  targetAngle[axis] = 90.0f;
}

// ---------- motion ----------

void updateServoPositions() {
  unsigned long now = millis();

  for (int i = 0; i < AXIS_COUNT; i++) {
    if (!attached[i]) continue;
    if (currentAngle[i] == targetAngle[i]) { lastMoveTime[i] = now; continue; }

    unsigned long dt = now - lastMoveTime[i];
    if (dt < 5) continue;   // ~200 Hz update cap keeps I2C traffic sane

    float maxStep = MAX_SPEED_DEG_PER_SEC * (dt / 1000.0f);
    float diff = targetAngle[i] - currentAngle[i];

    if (fabsf(diff) <= maxStep) currentAngle[i] = targetAngle[i];
    else currentAngle[i] += (diff > 0) ? maxStep : -maxStep;

    writeAxis(i, currentAngle[i]);
    lastMoveTime[i] = now;
  }
}

// ---------- parsing ----------

bool parseInteger(const String &s, long &result) {
  if (s.length() == 0) return false;
  int i = 0;
  bool negative = false;
  if (s[0] == '-') { negative = true; i = 1; }
  else if (s[0] == '+') { i = 1; }
  if (i == (int)s.length()) return false;

  long value = 0;
  for (; i < (int)s.length(); i++) {
    char c = s[i];
    if (c < '0' || c > '9') return false;
    value = value * 10 + (c - '0');
  }
  result = negative ? -value : value;
  return true;
}

void processCommand(const String &line) {
  int start = 0;
  for (int i = 0; i < AXIS_COUNT; i++) {
    int commaIndex = line.indexOf(',', start);
    String field;
    if (commaIndex >= 0) {
      field = line.substring(start, commaIndex);
      start = commaIndex + 1;
    } else {
      field = line.substring(start);
      start = line.length();
    }
    field.trim();

    if (field.length() == 0) {
      if (EMPTY_MEANS_NO_SIGNAL) releaseServo(i);
    } else {
      long value;
      if (parseInteger(field, value) && value >= 0 && value <= 180) {
        moveServo(i, (int)value);
      }
    }
    if (commaIndex < 0) break;
  }
}

// ---------- setup / loop ----------

void setup() {
  Serial.begin(SERIAL_BAUD);
  inputLine.reserve(MAX_LINE_LENGTH);

  // Base pin: known no-PWM state
  pinMode(BASE_PIN, OUTPUT);
  digitalWrite(BASE_PIN, LOW);
  ESP32PWM::allocateTimer(0);

  // PCA9685
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  pca.begin();
  pca.setOscillatorFrequency(PCA_OSC_HZ);
  pca.setPWMFreq(50);
  delay(10);
  for (uint8_t label = 1; label <= 16; label++) pcaOff(label);   // all outputs silent

  for (int i = 0; i < AXIS_COUNT; i++) {
    currentAngle[i] = 90.0f;
    targetAngle[i] = 90.0f;
  }

  Serial.println(F("Ready (ESP32+PCA9685). Format: Gripper,WristYaw,WristPitch,ArmB,ArmA,Base"));
  Serial.println(F("Arm A -> PCA 1, mirrored on PCA 2 | Base -> ESP32 GPIO"));
  Serial.print(F("Speed Control: "));
  Serial.print(MAX_SPEED_DEG_PER_SEC);
  Serial.println(F(" deg/sec"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    lastCharTime = millis();
    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) processCommand(inputLine);
      inputLine = "";
    } else if (inputLine.length() < MAX_LINE_LENGTH) {
      inputLine += c;
    }
  }

  if (inputLine.length() > 0 && millis() - lastCharTime >= LINE_TIMEOUT_MS) {
    processCommand(inputLine);
    inputLine = "";
  }

  updateServoPositions();
}
