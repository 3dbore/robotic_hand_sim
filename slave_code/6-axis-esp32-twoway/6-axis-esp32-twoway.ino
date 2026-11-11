/*
 * ELIOS 6-axis slave, two-way coordinate mode — ESP32 + PCA9685
 * --------------------------------------------------------------
 * Driven by arm-hil-toolkit.py over serial. Handles both control methods, so the
 * toolkit's switch works without reflashing:
 *   Stream   the toolkit streams six servo angles, exactly as in 6-axis-esp32.ino.
 *   Two-way  the toolkit sends one coordinate; this board solves the joints, runs the
 *            move, and reports when it has finished.
 *
 * Two-way commands (toolkit -> board), each followed by ",CC" (CRC-8 as below).
 * SEQ is 1..65535. Sending the same SEQ again never repeats the action, it only
 * repeats the answer, so a lost line can safely be sent again.
 *   "V,SEQ"                       start a session: stops any move, answers with the firmware id
 *   "C,SEQ,J,HOME,GAIN,SLO,SHI,QLO,QHI"
 *                                 mapping of sim joint J (0..4): servo = HOME + GAIN * angle,
 *                                 servo range SLO..SHI, joint limits QLO..QHI
 *   "H,SEQ,GRIP"                  go to the initial pose. Servos that are off start there.
 *   "M,SEQ,L,X,Y,Z,A,B,S,GRIP"    go to wrist position X Y Z (mm), pitch A, roll B (deg).
 *                                 L=1 straight line, L=0 straight in joint space.
 *                                 S = average speed in mm/s. GRIP = gripper servo degrees.
 *   "X,SEQ"                       stop now and hold
 *
 * Two-way answers (board -> toolkit), each followed by ",CC":
 *   "OK seq=N [fw=ID]"            V or C applied
 *   "ACK seq=N t=1.23"            H or M accepted, about 1.23 s left
 *   "DONE seq=N q=0.00,..."       H, M or X finished; q = the five joint angles commanded
 *   "ERR seq=N reason=R"          refused: REACH (target out of reach), PATH (the straight
 *                                 line leaves the reach), HOME (no H yet), CFG (no C yet),
 *                                 BUSY (another move is running), VALUE (bad number)
 * DONE means the commanded motion has ended and SETTLE_MS has passed. Nothing here
 * measures the shaft; see "What the checks can and cannot see".
 *
 * Stream commands (toolkit -> board):
 *   "F0,F1,F2,F3,F4,F5,CC\n"       any of these also ends two-way motion until the next H
 *    F0 Gripper | F1 Wrist Yaw | F2 Wrist Pitch | F3 Arm B | F4 Arm A (mirrored pair) | F5 Base
 *    Fn  degrees -36..216 with at most one decimal; empty field = release servo.
 *        0..180 is the standard 500-2500 us. The rest exists so an axis offset can
 *        carry a joint's range past it, for servos that physically travel further.
 *    CC  CRC-8 (poly 0x07, init 0) of everything before the last comma, two hex digits
 *   A frame is applied whole or not at all. To type frames by hand in a serial
 *   monitor, set REQUIRE_CRC to false and leave ",CC" off.
 *
 * Reports (board -> toolkit):
 *   "BOOT reason=BROWNOUT"   once at startup, why the chip reset
 *   "STAT rx=120 bad=0 i2c=0 fix=0 reinit=0 spike=0 out=10.0,90.0,90.0,180.0,90.0,-"   every second
 *     rx      frames applied
 *     bad     frames dropped (checksum, shape, value, cut off, too long)
 *     i2c     failed I2C transactions
 *     fix     PCA output registers found wrong and rewritten
 *     reinit  times the PCA was found reset/asleep (or the base PWM lost) and restored
 *     spike   out-of-range jumps held back and never confirmed (valid checksum, wrong value)
 *     out     per-axis output degrees read back from the hardware; "-" = no pulses
 *
 * What the checks can and cannot see: an MG996R gives the ESP32 no position
 * signal. This firmware verifies everything up to the servo connector (frames,
 * I2C, PCA registers, resets). It cannot see the shaft, so a joint that was
 * pushed, stalled, or lost servo power reads as healthy. Catching those needs a
 * sensor on the joint, such as an AS5600 magnetic encoder.
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
 *   - "ESP32Servo" by Kevin Harrington / madhephaestus, for its ESP32PWM class
 *   The PCA9685 is driven directly over Wire, so the registers can be read back.
 *   The base uses ESP32PWM rather than Servo, which refuses pulses outside 500-2500 us.
 */

#include <Wire.h>
#include <ESP32PWM.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_system.h>
#include "kinematics.h"

// ================= CONFIGURATION =================
const long SERIAL_BAUD = 9600;          // must match SERIAL_BAUD in hardware_mapping.py

// --- Direct-wired base servo ---
const int BASE_PIN = 13;                // change to the GPIO you actually used

// --- PCA9685 ---
const uint8_t PCA_ADDR = 0x40;
const int I2C_SDA = 21;
const int I2C_SCL = 22;
const uint32_t I2C_HZ = 100000;         // slower is more tolerant of servo noise on the bus
const uint16_t I2C_TIMEOUT_MS = 20;
const uint32_t PCA_OSC_HZ = 27000000;   // trim if pulses measure off (typ. 25–27 MHz)
const uint32_t PWM_HZ = 50;

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

// MG996R pulse range: these pulses are 0 and 180 degrees
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Furthest command accepted, which works out to 100 and 2900 us. Keep in step with
// PULSE_LIMIT_MIN/MAX_DEG in hardware_mapping.py.
const int ANGLE_LIMIT_MIN = -36;
const int ANGLE_LIMIT_MAX = 216;

// Speed limit (same as Uno firmware)
const float MAX_SPEED_DEG_PER_SEC = 60.0f;

// Spike gate. A frame that passes its checksum can still carry a wrong value (a
// "180" that arrived as "18"). A target further than SPIKE_GATE_DEG from the last
// accepted one is held back until SPIKE_CONFIRM_FRAMES frames in a row agree on it
// (within SPIKE_MATCH_DEG). A real move costs one extra frame (50 ms at 20 Hz); a
// one-frame glitch never reaches the servo. Set SPIKE_CONFIRM_FRAMES to 1 to disable.
// The gate must stay larger than the biggest honest step between two frames.
const float SPIKE_GATE_DEG = 20.0f;
const float SPIKE_MATCH_DEG = 3.0f;
const int SPIKE_CONFIRM_FRAMES = 2;

// Low-pass on the accepted target, applied before the speed limit. Smooths small
// jitter and the stair-steps of a 20 Hz stream. 0 turns it off.
const float SMOOTHING_TAU_MS = 80.0f;
const float SMOOTHING_SNAP_DEG = 0.05f;   // closer than this, land exactly on target

// true  = empty field releases the servo (no PWM pulse)
// false = empty field keeps the servo holding its last position
const bool EMPTY_MEANS_NO_SIGNAL = true;

const bool REQUIRE_CRC = true;

// A line that stops arriving this long before its newline is thrown away, never run.
const unsigned long LINE_TIMEOUT_MS = 100;
const int MAX_LINE_LENGTH = 96;

// Two-way mode. Keep FIRMWARE_ID in step with TWOWAY_FIRMWARE_ID in waypoint.py.
const char FIRMWARE_ID[] = "twoway-1";
const unsigned long SETTLE_MS = 150;    // pause after a move, before DONE, so the arm can come to rest

const unsigned long MOTION_PERIOD_MS = 20;    // servos only take a new pulse every 20 ms anyway
const unsigned long MAX_MOTION_DT_MS = 100;   // a stalled loop must not turn into a jump
const unsigned long VERIFY_PERIOD_MS = 20;    // one PCA channel per pass
const unsigned long HEALTH_PERIOD_MS = 500;
const unsigned long STAT_PERIOD_MS = 1000;
// Servos draw a big inrush when they first get pulses. Starting them one at a time
// keeps that from sagging the supply and resetting the board.
const unsigned long ATTACH_STAGGER_MS = 250;
const int I2C_ERRORS_BEFORE_RECOVERY = 10;
const unsigned long I2C_RECOVERY_PERIOD_MS = 1000;
// =================================================

// PCA9685 registers
const uint8_t REG_MODE1 = 0x00;
const uint8_t REG_LED0_ON_L = 0x06;
const uint8_t REG_PRESCALE = 0xFE;
const uint8_t MODE1_RESTART = 0x80;
const uint8_t MODE1_AI = 0x20;      // auto-increment, required by the block reads/writes below
const uint8_t MODE1_SLEEP = 0x10;
const int PCA_CHANNELS = 16;
const uint16_t TICKS_FULL_OFF = 4096;   // the OFF register's full-off bit: line held LOW

const uint8_t PCA_PRESCALE = (uint8_t)((float)PCA_OSC_HZ / (PWM_HZ * 4096.0f) + 0.5f) - 1;
const float US_PER_TICK = 1000000.0f * (PCA_PRESCALE + 1) / PCA_OSC_HZ;

const int BASE_PWM_BITS = 14;           // 14 bits exist on every ESP32 variant; 0.11 deg per tick
const float BASE_US_PER_TICK = (1000000.0f / PWM_HZ) / (1UL << BASE_PWM_BITS);

ESP32PWM basePwm;

// Started one at a time from a queue, in this order: heaviest load first.
const uint8_t ATTACH_ORDER[AXIS_COUNT] = { BASE, ARM_A, ARM_B, WRIST_PITCH, WRIST_YAW, GRIPPER };

bool attached[AXIS_COUNT] = {false};
bool pendingAttach[AXIS_COUNT] = {false};
float currentAngle[AXIS_COUNT];
float targetAngle[AXIS_COUNT];
float smoothAngle[AXIS_COUNT];          // targetAngle after the low-pass, before the speed limit

bool haveAccepted[AXIS_COUNT] = {false};   // false until the first frame after start/release
float candidateAngle[AXIS_COUNT];          // out-of-gate value waiting for confirmation
int candidateCount[AXIS_COUNT] = {0};

uint16_t expectedTicks[PCA_CHANNELS];   // what each channel should hold
uint16_t readbackTicks[PCA_CHANNELS];   // what it last held when read

char lineBuf[MAX_LINE_LENGTH + 1];
int lineLen = 0;
bool lineOverflow = false;
unsigned long lastCharTime = 0;

unsigned long lastMotionMs = 0;
unsigned long lastVerifyMs = 0;
unsigned long lastHealthMs = 0;
unsigned long lastStatMs = 0;
unsigned long lastAttachMs = 0;
unsigned long lastRecoveryMs = 0;
int verifyChannel = 0;

unsigned long countRx = 0;
unsigned long countBad = 0;
unsigned long countI2c = 0;
unsigned long countFix = 0;
unsigned long countReinit = 0;
unsigned long countSpike = 0;
int consecutiveI2cErrors = 0;

// Two-way mode: sim joints J1..J5 and the axis each one drives.
const int JOINT_AXIS[kin::JOINT_COUNT] = { BASE, ARM_A, ARM_B, WRIST_PITCH, WRIST_YAW };

enum MotionState { MOTION_IDLE, MOTION_HOMING, MOTION_MOVING, MOTION_SETTLING };
MotionState motionState = MOTION_IDLE;

bool jointConfigured[kin::JOINT_COUNT] = {false};
float jointHome[kin::JOINT_COUNT];
float jointGain[kin::JOINT_COUNT];
float jointServoLo[kin::JOINT_COUNT];
float jointServoHi[kin::JOINT_COUNT];
double jointLo[kin::JOINT_COUNT];
double jointHi[kin::JOINT_COUNT];
double jointRatio[kin::JOINT_COUNT];

bool homed = false;                     // jointQ describes where the servos are
double jointQ[kin::JOINT_COUNT] = {0};
kin::Plan movePlan;
double moveElapsed = 0.0;
unsigned long lastPlanMs = 0;
unsigned long settleStartMs = 0;

long activeSeq = -1;                    // the H or M being carried out
long lastSeq = -1;                      // the last command answered for good, and that answer
char lastReply[MAX_LINE_LENGTH];

// ---------- PCA9685 ----------

static inline uint8_t pcaChannel(uint8_t label) { return label - 1; }

static bool noteI2c(bool ok) {
  if (ok) {
    consecutiveI2cErrors = 0;
  } else {
    countI2c++;
    consecutiveI2cErrors++;
  }
  return ok;
}

static bool pcaWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return noteI2c(Wire.endTransmission() == 0);
}

static bool pcaRead(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return noteI2c(false);
  if (Wire.requestFrom(PCA_ADDR, count) != count) return noteI2c(false);
  for (uint8_t i = 0; i < count; i++) data[i] = Wire.read();
  return noteI2c(true);
}

// All four registers go out in one transaction; the PCA applies them together at the STOP.
static bool pcaWriteChannel(uint8_t channel) {
  uint16_t ticks = expectedTicks[channel];
  Wire.beginTransmission(PCA_ADDR);
  Wire.write((uint8_t)(REG_LED0_ON_L + 4 * channel));
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)(ticks & 0xFF));
  Wire.write((uint8_t)(ticks >> 8));
  return noteI2c(Wire.endTransmission() == 0);
}

static void pcaSetChannel(uint8_t channel, uint16_t ticks) {
  expectedTicks[channel] = ticks;
  pcaWriteChannel(channel);   // a failed write is put right by the next verify pass
}

static void pcaConfigure() {
  pcaWriteReg(REG_MODE1, MODE1_SLEEP);      // the prescaler can only be set asleep
  pcaWriteReg(REG_PRESCALE, PCA_PRESCALE);
  pcaWriteReg(REG_MODE1, MODE1_AI);
  delay(1);                                 // oscillator start-up (500 us max)
  pcaWriteReg(REG_MODE1, MODE1_AI | MODE1_RESTART);
}

static void pcaWriteAllChannels() {
  for (int ch = 0; ch < PCA_CHANNELS; ch++) pcaWriteChannel(ch);
}

// Checks one channel per call, so the whole chip is covered about three times a second.
static void verifyNextChannel() {
  uint8_t channel = verifyChannel;
  verifyChannel = (verifyChannel + 1) % PCA_CHANNELS;

  uint8_t raw[4];
  if (!pcaRead(REG_LED0_ON_L + 4 * channel, raw, 4)) return;   // says nothing about the register
  uint16_t on = raw[0] | (raw[1] << 8);
  uint16_t off = raw[2] | (raw[3] << 8);
  readbackTicks[channel] = off;
  if (on != 0 || off != expectedTicks[channel]) {
    countFix++;
    pcaWriteChannel(channel);
  }
}

// ---------- low-level output ----------

static inline float angleToUs(float angle) {
  if (angle < ANGLE_LIMIT_MIN) angle = ANGLE_LIMIT_MIN;   // the mirror trim can land past the limit
  if (angle > ANGLE_LIMIT_MAX) angle = ANGLE_LIMIT_MAX;
  return SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180.0f);
}

static inline float usToAngle(float us) {
  return (us - SERVO_MIN_US) * 180.0f / (SERVO_MAX_US - SERVO_MIN_US);
}

static inline uint16_t angleToTicks(float angle) {
  return (uint16_t)lroundf(angleToUs(angle) / US_PER_TICK);
}

// Writes a (fractional) angle to the physical motor(s) behind one axis.
// PCA9685 has ~4.9 us resolution, so half-degree steps are real at the output.
static void writeAxis(int axis, float angle) {
  if (axis == BASE) {
    basePwm.write((uint32_t)lroundf(angleToUs(angle) / BASE_US_PER_TICK));
    return;
  }
  pcaSetChannel(pcaChannel(PCA_LABEL[axis]), angleToTicks(angle));

  if (axis == ARM_A) {
    float mirrored = 180.0f - angle + ARM_A_MIRROR_TRIM_DEG;
    pcaSetChannel(pcaChannel(ARM_A_MIRROR_LABEL), angleToTicks(mirrored));
  }
}

// ---------- attach / release ----------

static void attachBase() {
  basePwm.attachPin(BASE_PIN, PWM_HZ, BASE_PWM_BITS);
}

static void moveServo(int axis, float angle) {
  if (!attached[axis] && !pendingAttach[axis]) {
    // Like the Uno firmware: the first command after release is where it starts
    // (we don't know where the unpowered servo was left). Later frames that arrive
    // before its turn in the queue only move the target, so they stay speed-limited.
    pendingAttach[axis] = true;
    currentAngle[axis] = angle;
    smoothAngle[axis] = angle;
  }
  targetAngle[axis] = angle;
}

// Spike gate in front of moveServo; see SPIKE_GATE_DEG.
static void acceptTarget(int axis, float angle) {
  if (!haveAccepted[axis] || fabsf(angle - targetAngle[axis]) <= SPIKE_GATE_DEG) {
    if (candidateCount[axis] > 0) countSpike++;   // the held value was a one-off
    candidateCount[axis] = 0;
    haveAccepted[axis] = true;
    moveServo(axis, angle);
    return;
  }
  if (candidateCount[axis] > 0 && fabsf(angle - candidateAngle[axis]) <= SPIKE_MATCH_DEG) {
    candidateCount[axis]++;
  } else {
    if (candidateCount[axis] > 0) countSpike++;   // the previous candidate never repeated
    candidateAngle[axis] = angle;
    candidateCount[axis] = 1;
  }
  if (candidateCount[axis] >= SPIKE_CONFIRM_FRAMES) {
    candidateCount[axis] = 0;
    moveServo(axis, angle);
  }
}

static void serviceAttachQueue(unsigned long now) {
  if (now - lastAttachMs < ATTACH_STAGGER_MS) return;
  for (int i = 0; i < AXIS_COUNT; i++) {
    int axis = ATTACH_ORDER[i];
    if (!pendingAttach[axis]) continue;
    pendingAttach[axis] = false;
    if (axis == BASE) attachBase();
    attached[axis] = true;
    writeAxis(axis, currentAngle[axis]);
    lastAttachMs = now;
    return;
  }
}

static void releaseServo(int axis) {
  pendingAttach[axis] = false;
  if (!attached[axis]) return;

  if (axis == BASE) {
    basePwm.detachPin(BASE_PIN);
    pinMode(BASE_PIN, OUTPUT);
    digitalWrite(BASE_PIN, LOW);
  } else {
    pcaSetChannel(pcaChannel(PCA_LABEL[axis]), TICKS_FULL_OFF);
    if (axis == ARM_A) pcaSetChannel(pcaChannel(ARM_A_MIRROR_LABEL), TICKS_FULL_OFF);
  }

  attached[axis] = false;
  currentAngle[axis] = 90.0f;
  targetAngle[axis] = 90.0f;
  smoothAngle[axis] = 90.0f;
}

// Forgets the spike gate's history: the next frame is where the servo starts.
static void forgetAxis(int axis) {
  haveAccepted[axis] = false;
  candidateCount[axis] = 0;
}

// ---------- motion ----------

static void updateServoPositions(unsigned long now) {
  unsigned long dt = now - lastMotionMs;
  if (dt < MOTION_PERIOD_MS) return;
  lastMotionMs = now;
  if (dt > MAX_MOTION_DT_MS) dt = MAX_MOTION_DT_MS;
  float maxStep = MAX_SPEED_DEG_PER_SEC * (dt / 1000.0f);
  float blend = (SMOOTHING_TAU_MS > 0.0f) ? 1.0f - expf(-(float)dt / SMOOTHING_TAU_MS) : 1.0f;

  for (int i = 0; i < AXIS_COUNT; i++) {
    if (!attached[i] || (currentAngle[i] == targetAngle[i] && smoothAngle[i] == targetAngle[i])) continue;

    smoothAngle[i] += (targetAngle[i] - smoothAngle[i]) * blend;
    if (fabsf(targetAngle[i] - smoothAngle[i]) < SMOOTHING_SNAP_DEG) smoothAngle[i] = targetAngle[i];

    float before = currentAngle[i];
    float diff = smoothAngle[i] - currentAngle[i];
    if (fabsf(diff) <= maxStep) currentAngle[i] = smoothAngle[i];
    else currentAngle[i] += (diff > 0) ? maxStep : -maxStep;

    if (currentAngle[i] != before) writeAxis(i, currentAngle[i]);
  }
}

// ---------- health ----------

// A PCA that lost power comes back asleep with default registers and every output off.
static void checkHealth() {
  uint8_t mode1, prescale;
  if (pcaRead(REG_MODE1, &mode1, 1) && pcaRead(REG_PRESCALE, &prescale, 1)) {
    if ((mode1 & MODE1_SLEEP) || !(mode1 & MODE1_AI) || prescale != PCA_PRESCALE) {
      countReinit++;
      pcaConfigure();
      pcaWriteAllChannels();
    }
  }

  if (attached[BASE]) {
    if (!basePwm.attached()) {
      countReinit++;
      attachBase();
    }
    writeAxis(BASE, currentAngle[BASE]);
  }
}

// A slave can hold SDA low mid-byte after a glitch; clocking it out frees the bus.
static void recoverI2cBus() {
  Wire.end();
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
  }
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  consecutiveI2cErrors = 0;
}

static const char *resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

static void printStat() {
  char buf[160];
  int n = snprintf(buf, sizeof(buf), "STAT rx=%lu bad=%lu i2c=%lu fix=%lu reinit=%lu spike=%lu out=",
                   countRx, countBad, countI2c, countFix, countReinit, countSpike);
  for (int i = 0; i < AXIS_COUNT; i++) {
    const char *sep = (i + 1 < AXIS_COUNT) ? "," : "";
    // Full-off in the readback: no pulses seen yet, or the channel really is silent.
    uint16_t ticks = (i == BASE) ? 0 : readbackTicks[pcaChannel(PCA_LABEL[i])];
    if (!attached[i] || ticks >= TICKS_FULL_OFF) {
      n += snprintf(buf + n, sizeof(buf) - n, "-%s", sep);
      continue;
    }
    float us = (i == BASE) ? basePwm.read() * BASE_US_PER_TICK : ticks * US_PER_TICK;
    n += snprintf(buf + n, sizeof(buf) - n, "%.1f%s", usToAngle(us), sep);
  }
  Serial.println(buf);
}

// ---------- checksums ----------

static uint8_t crc8(const char *data, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    crc ^= (uint8_t)data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// ---------- two-way motion ----------

static void sendFramed(const char *text) {
  char buf[MAX_LINE_LENGTH + 8];
  snprintf(buf, sizeof(buf), "%s,%02X", text, crc8(text, strlen(text)));
  Serial.println(buf);
}

// A final answer: kept, so a repeat of the same SEQ gets it again.
static void finishCommand(long seq, const char *text) {
  lastSeq = seq;
  strncpy(lastReply, text, sizeof(lastReply) - 1);
  lastReply[sizeof(lastReply) - 1] = '\0';
  sendFramed(text);
}

static void sendError(long seq, const char *reason) {
  char text[MAX_LINE_LENGTH];
  snprintf(text, sizeof(text), "ERR seq=%ld reason=%s", seq, reason);
  finishCommand(seq, text);
}

static void sendDone(long seq) {
  char text[MAX_LINE_LENGTH];
  snprintf(text, sizeof(text), "DONE seq=%ld q=%.2f,%.2f,%.2f,%.2f,%.2f", seq,
           jointQ[0], jointQ[1], jointQ[2], jointQ[3], jointQ[4]);
  finishCommand(seq, text);
}

static float jointToServo(int joint, double angle) {
  float servo = jointHome[joint] + jointGain[joint] * (float)angle;
  if (servo < jointServoLo[joint]) return jointServoLo[joint];
  if (servo > jointServoHi[joint]) return jointServoHi[joint];
  return servo;
}

// Plan output goes straight to the servo: the plan is already smooth and within the
// speed limit, so the spike gate, low-pass and speed limit are skipped.
static void writeJoints() {
  for (int j = 0; j < kin::JOINT_COUNT; j++) {
    int axis = JOINT_AXIS[j];
    float angle = jointToServo(j, jointQ[j]);
    currentAngle[axis] = angle;
    smoothAngle[axis] = angle;
    targetAngle[axis] = angle;
    writeAxis(axis, angle);
  }
}

// Seconds until the speed-limited axes (the gripper in a move, all of them in a homing) arrive.
static float slewRemaining() {
  float seconds = 0.0f;
  int waiting = 0;
  for (int i = 0; i < AXIS_COUNT; i++) {
    if (pendingAttach[i]) waiting++;
    float s = fabsf(targetAngle[i] - currentAngle[i]) / MAX_SPEED_DEG_PER_SEC;
    if (s > seconds) seconds = s;
  }
  return seconds + waiting * (ATTACH_STAGGER_MS / 1000.0f);
}

static bool axesArrived() {
  for (int i = 0; i < AXIS_COUNT; i++) {
    if (pendingAttach[i] || !attached[i] || currentAngle[i] != targetAngle[i]) return false;
  }
  return true;
}

static void sendAck() {
  float left = slewRemaining();
  if (motionState == MOTION_MOVING) {
    float planLeft = (float)(movePlan.duration - moveElapsed);
    if (planLeft > left) left = planLeft;
  }
  left += SETTLE_MS / 1000.0f;
  char text[MAX_LINE_LENGTH];
  snprintf(text, sizeof(text), "ACK seq=%ld t=%.2f", activeSeq, left);
  sendFramed(text);
}

// Ends any two-way motion. Every axis holds the place it has reached.
static void stopMotion() {
  if (motionState == MOTION_IDLE) return;
  if (motionState == MOTION_HOMING) homed = false;   // the servos never got to jointQ
  motionState = MOTION_IDLE;
  activeSeq = -1;
  for (int i = 0; i < AXIS_COUNT; i++) {
    targetAngle[i] = currentAngle[i];
    smoothAngle[i] = currentAngle[i];
  }
}

static void serviceMotion(unsigned long now) {
  if (motionState == MOTION_IDLE) return;

  if (motionState == MOTION_HOMING) {
    if (axesArrived()) {
      homed = true;
      motionState = MOTION_SETTLING;
      settleStartMs = now;
    }
  } else if (motionState == MOTION_MOVING) {
    unsigned long dt = now - lastPlanMs;
    if (dt < MOTION_PERIOD_MS) return;
    lastPlanMs = now;
    if (dt > MAX_MOTION_DT_MS) dt = MAX_MOTION_DT_MS;   // a stalled loop must not turn into a jump
    moveElapsed += dt / 1000.0;
    kin::planAngles(movePlan, moveElapsed, jointLo, jointHi, jointQ);
    writeJoints();
    if (moveElapsed >= movePlan.duration && axesArrived()) {
      motionState = MOTION_SETTLING;
      settleStartMs = now;
    }
  } else if (now - settleStartMs >= SETTLE_MS) {
    long seq = activeSeq;
    motionState = MOTION_IDLE;
    activeSeq = -1;
    sendDone(seq);
  }
}

// ---------- two-way commands ----------

static bool parseNumber(const char *s, double &result) {
  if (*s == '\0') return false;
  char *end;
  result = strtod(s, &end);
  return *end == '\0' && isfinite(result);
}

static bool validServoAngle(double angle) {
  return angle >= ANGLE_LIMIT_MIN && angle <= ANGLE_LIMIT_MAX;
}

static void commandConfig(long seq, char **f) {
  double j, home, gain, servoLo, servoHi, lo, hi;
  if (!parseNumber(f[2], j) || !parseNumber(f[3], home) || !parseNumber(f[4], gain) ||
      !parseNumber(f[5], servoLo) || !parseNumber(f[6], servoHi) ||
      !parseNumber(f[7], lo) || !parseNumber(f[8], hi) ||
      j < 0 || j >= kin::JOINT_COUNT || j != floor(j) || gain == 0.0 || servoLo > servoHi || lo > hi ||
      !validServoAngle(servoLo) || !validServoAngle(servoHi)) {
    sendError(seq, "VALUE");
    return;
  }
  if (motionState != MOTION_IDLE) {
    sendError(seq, "BUSY");
    return;
  }
  int joint = (int)j;
  jointHome[joint] = (float)home;
  jointGain[joint] = (float)gain;
  jointServoLo[joint] = (float)servoLo;
  jointServoHi[joint] = (float)servoHi;
  jointLo[joint] = lo;
  jointHi[joint] = hi;
  jointRatio[joint] = fabs(gain);
  jointConfigured[joint] = true;
  homed = false;

  char text[MAX_LINE_LENGTH];
  snprintf(text, sizeof(text), "OK seq=%ld", seq);
  finishCommand(seq, text);
}

static void commandHome(long seq, char **f) {
  double grip;
  if (!parseNumber(f[2], grip) || !validServoAngle(grip)) {
    sendError(seq, "VALUE");
    return;
  }
  for (int j = 0; j < kin::JOINT_COUNT; j++) {
    if (!jointConfigured[j]) {
      sendError(seq, "CFG");
      return;
    }
  }
  if (motionState != MOTION_IDLE) {
    sendError(seq, "BUSY");
    return;
  }
  for (int j = 0; j < kin::JOINT_COUNT; j++) {
    jointQ[j] = 0.0;
    moveServo(JOINT_AXIS[j], jointToServo(j, 0.0));
    haveAccepted[JOINT_AXIS[j]] = true;
    candidateCount[JOINT_AXIS[j]] = 0;
  }
  moveServo(GRIPPER, (float)grip);
  haveAccepted[GRIPPER] = true;
  candidateCount[GRIPPER] = 0;

  homed = false;
  activeSeq = seq;
  motionState = MOTION_HOMING;
  sendAck();
}

static void commandMove(long seq, char **f) {
  double linear, speed, grip;
  kin::Pose target;
  if (!parseNumber(f[2], linear) || !parseNumber(f[3], target.x) || !parseNumber(f[4], target.y) ||
      !parseNumber(f[5], target.z) || !parseNumber(f[6], target.a) || !parseNumber(f[7], target.b) ||
      !parseNumber(f[8], speed) || !parseNumber(f[9], grip) ||
      speed < 0.0 || !validServoAngle(grip)) {
    sendError(seq, "VALUE");
    return;
  }
  if (motionState != MOTION_IDLE) {
    sendError(seq, "BUSY");
    return;
  }
  if (!homed) {
    sendError(seq, "HOME");
    return;
  }
  kin::PlanResult result = kin::planMove(jointQ, target, linear != 0.0, speed, jointLo, jointHi,
                                         jointRatio, MAX_SPEED_DEG_PER_SEC, movePlan);
  if (result != kin::PLAN_OK) {
    sendError(seq, result == kin::PLAN_REACH ? "REACH" : "PATH");
    return;
  }
  moveServo(GRIPPER, (float)grip);
  moveElapsed = 0.0;
  lastPlanMs = millis();
  activeSeq = seq;
  motionState = MOTION_MOVING;
  sendAck();
}

// `line` is a whole two-way command with its checksum, and may be edited.
static bool processCommand(char *line, int len) {
  if (len < 3 || line[len - 3] != ',') return false;
  int hi = hexDigit(line[len - 2]);
  int lo = hexDigit(line[len - 1]);
  if (hi < 0 || lo < 0) return false;
  int payloadLen = len - 3;
  if (crc8(line, payloadLen) != (uint8_t)((hi << 4) | lo)) return false;
  line[payloadLen] = '\0';

  const int MAX_FIELDS = 10;
  char *fields[MAX_FIELDS];
  int count = 0;
  char *p = line;
  while (true) {
    if (count == MAX_FIELDS) return false;
    fields[count++] = p;
    char *comma = strchr(p, ',');
    if (comma == NULL) break;
    *comma = '\0';
    p = comma + 1;
  }

  char command = fields[0][0];
  int expected;
  switch (command) {
    case 'V': case 'X': expected = 2; break;
    case 'H':           expected = 3; break;
    case 'C':           expected = 9; break;
    case 'M':           expected = 10; break;
    default:            return false;
  }
  double seqValue;
  if (fields[0][1] != '\0' || count != expected || !parseNumber(fields[1], seqValue) ||
      seqValue < 1 || seqValue > 65535 || seqValue != floor(seqValue)) {
    return false;
  }
  long seq = (long)seqValue;

  if (command == 'V') {             // a new session: nothing before it can be repeated
    stopMotion();
    char text[MAX_LINE_LENGTH];
    snprintf(text, sizeof(text), "OK seq=%ld fw=%s", seq, FIRMWARE_ID);
    finishCommand(seq, text);
    return true;
  }
  if (seq == activeSeq) {           // repeat of the running command
    sendAck();
    return true;
  }
  if (seq == lastSeq) {             // repeat of a finished one
    sendFramed(lastReply);
    return true;
  }

  switch (command) {
    case 'C': commandConfig(seq, fields); break;
    case 'H': commandHome(seq, fields); break;
    case 'M': commandMove(seq, fields); break;
    case 'X':
      stopMotion();
      sendDone(seq);
      break;
  }
  return true;
}

// ---------- stream frames ----------

// Accepts "d", "dd", "ddd", each with an optional leading "-" and an optional ".d";
// range ANGLE_LIMIT_MIN..ANGLE_LIMIT_MAX.
static bool parseAngle(const char *s, int len, float &result) {
  bool negative = len > 0 && s[0] == '-';
  int first = negative ? 1 : 0;
  int i = first;
  int whole = 0;
  while (i < len && s[i] >= '0' && s[i] <= '9') {
    if (i - first == 3) return false;
    whole = whole * 10 + (s[i] - '0');
    i++;
  }
  if (i == first) return false;

  int tenths = 0;
  if (i < len) {
    if (s[i] != '.' || i + 2 != len || s[i + 1] < '0' || s[i + 1] > '9') return false;
    tenths = s[i + 1] - '0';
  }
  int value = whole * 10 + tenths;
  if (negative) value = -value;
  if (value < ANGLE_LIMIT_MIN * 10 || value > ANGLE_LIMIT_MAX * 10) return false;
  result = value / 10.0f;
  return true;
}

// Checks the whole frame before touching any axis.
static bool processLine(char *line, int len) {
  if (line[0] >= 'A' && line[0] <= 'Z') return processCommand(line, len);

  int payloadLen = len;
  int commas = 0;
  for (int i = 0; i < len; i++) commas += (line[i] == ',');

  if (commas == AXIS_COUNT) {
    if (len < 3 || line[len - 3] != ',') return false;
    int hi = hexDigit(line[len - 2]);
    int lo = hexDigit(line[len - 1]);
    if (hi < 0 || lo < 0) return false;
    payloadLen = len - 3;
    if (crc8(line, payloadLen) != (uint8_t)((hi << 4) | lo)) return false;
  } else if (REQUIRE_CRC || commas != AXIS_COUNT - 1) {
    return false;
  }

  float values[AXIS_COUNT];
  bool present[AXIS_COUNT];
  int start = 0;
  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    int end = start;
    while (end < payloadLen && line[end] != ',') end++;
    present[axis] = end > start;
    if (present[axis] && !parseAngle(line + start, end - start, values[axis])) return false;
    start = end + 1;
  }

  // The toolkit has gone back to streaming, or released the servos.
  stopMotion();
  homed = false;

  for (int axis = 0; axis < AXIS_COUNT; axis++) {
    if (present[axis]) {
      acceptTarget(axis, values[axis]);
    } else if (EMPTY_MEANS_NO_SIGNAL) {
      releaseServo(axis);
      forgetAxis(axis);
    }
  }
  return true;
}

static void readSerial(unsigned long now) {
  while (Serial.available()) {
    char c = Serial.read();
    lastCharTime = now;
    if (c == '\n' || c == '\r') {
      if (lineOverflow) {
        countBad++;
      } else if (lineLen > 0) {
        if (processLine(lineBuf, lineLen)) countRx++;
        else countBad++;
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < MAX_LINE_LENGTH) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }

  // Half a line is not a command: "90,9" could be the start of "90,90,...".
  if ((lineLen > 0 || lineOverflow) && now - lastCharTime >= LINE_TIMEOUT_MS) {
    countBad++;
    lineLen = 0;
    lineOverflow = false;
  }
}

// ---------- setup / loop ----------

void setup() {
  Serial.setTxBufferSize(256);   // reports must queue, not stall the control loop
  Serial.begin(SERIAL_BAUD);
  Serial.print("BOOT reason=");
  Serial.println(resetReasonName());

  // Base pin: known no-PWM state
  pinMode(BASE_PIN, OUTPUT);
  digitalWrite(BASE_PIN, LOW);
  ESP32PWM::allocateTimer(0);

  for (int ch = 0; ch < PCA_CHANNELS; ch++) {
    expectedTicks[ch] = TICKS_FULL_OFF;   // all outputs silent
    readbackTicks[ch] = TICKS_FULL_OFF;
  }
  for (int i = 0; i < AXIS_COUNT; i++) {
    currentAngle[i] = 90.0f;
    targetAngle[i] = 90.0f;
  }

  // PCA9685. If it isn't answering yet, checkHealth() configures it once it does.
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  pcaConfigure();
  delay(10);
  pcaWriteAllChannels();

  Serial.println("Ready (ESP32+PCA9685, stream + two-way). Stream format: Gripper,WristYaw,WristPitch,ArmB,ArmA,Base,CRC8");
  Serial.println("Arm A -> PCA 1, mirrored on PCA 2 | Base -> ESP32 GPIO");
  Serial.print("Speed Control: ");
  Serial.print(MAX_SPEED_DEG_PER_SEC);
  Serial.println(" deg/sec");
}

void loop() {
  unsigned long now = millis();

  readSerial(now);
  serviceAttachQueue(now);
  serviceMotion(now);
  updateServoPositions(now);

  if (now - lastVerifyMs >= VERIFY_PERIOD_MS) {
    lastVerifyMs = now;
    verifyNextChannel();
  }
  if (now - lastHealthMs >= HEALTH_PERIOD_MS) {
    lastHealthMs = now;
    checkHealth();
  }
  if (consecutiveI2cErrors >= I2C_ERRORS_BEFORE_RECOVERY &&
      now - lastRecoveryMs >= I2C_RECOVERY_PERIOD_MS) {
    lastRecoveryMs = now;
    recoverI2cBus();
  }
  if (now - lastStatMs >= STAT_PERIOD_MS) {
    lastStatMs = now;
    printStat();
  }
}
