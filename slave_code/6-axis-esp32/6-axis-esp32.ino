/*
 * ELIOS 6-axis slave — ESP32 + PCA9685
 * ------------------------------------
 * Driven by arm-hil-toolkit.py over serial.
 *
 * Commands (toolkit -> board):
 *   "F0,F1,F2,F3,F4,F5,CC\n"
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
#include <string.h>
#include <esp_system.h>

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
  8,    // WRIST_PITCH
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

// Stream motion, per axis in serial field order. Each axis ramps its speed up and
// down at its acceleration limit, so a move starts and stops gently instead of
// slamming the arm's momentum into the gear backlash (which is what makes it ring).
// The heavy arm joints get the lowest limits. Lower a joint's numbers if it still
// oscillates at the end of a move; raise them if it lags too far behind the sim.
//                                                    Grip   Yaw  Pitch  ArmB  ArmA  Base
const float AXIS_MAX_SPEED_DEG_PER_SEC[AXIS_COUNT]  = {  90,   45,   45,   30,   30,   40 };
const float AXIS_MAX_ACCEL_DEG_PER_SEC2[AXIS_COUNT] = { 300,  120,  120,   60,   60,   90 };

// Anti-backlash approach: every move ends travelling in the axis's APPROACH_DIR
// (+1 = increasing servo angle, -1 = decreasing, 0 = off), so the gear slack is always
// taken up on the same side and the joint stops at the same point every time. A move
// the other way runs APPROACH_OVERSHOOT_DEG past its target and comes back.
// For a joint that carries weight, pick the direction that LIFTS it: gravity then
// holds the slack closed. Check each one on the real arm, with the motors powered:
// jog the servo angle up and see whether the joint rises. Don't infer it from the sign
// in hardware_mapping.py: calibration/axis_directions.json can override that sign.
// The gripper is off: overshooting it would squeeze or bump its end stop.
// Base: a servo through a planetary gear, which has play too.
//                                   Grip  Yaw  Pitch  ArmB  ArmA  Base
const int APPROACH_DIR[AXIS_COUNT] = {  0,   +1,   +1,   +1,   -1,   +1 };
const float APPROACH_OVERSHOOT_DEG = 3.0f;   // a bit more than the play you can feel by hand
// The approach runs once, after the target has stopped changing for this long. A slow
// stream move arrives as 0.5 deg steps; dipping past every one of them made the joint
// saw back and forth by APPROACH_OVERSHOOT_DEG for the whole move.
const unsigned long APPROACH_SETTLE_MS = 250;

// Hold deadband: an axis at rest ignores a new stream target closer than this, so the
// servo isn't nudged back and forth inside its backlash while it should be holding.
// Just over one 0.5 deg command step. A lone smaller change is jitter and is never
// carried out; the same value in two frames in a row is meant (the toolkit sends an
// offset jog twice, and its heartbeat repeats a pose that has settled). 0 turns this off.
const float HOLD_DEADBAND_DEG = 0.6f;

// Spike gate. A frame that passes its checksum can still carry a wrong value (a
// "180" that arrived as "18"). A target further than SPIKE_GATE_DEG from the last
// accepted one is held back until SPIKE_CONFIRM_FRAMES frames in a row agree on it
// (within SPIKE_MATCH_DEG). A real move costs one extra frame (50 ms at 20 Hz); a
// one-frame glitch never reaches the servo. Set SPIKE_CONFIRM_FRAMES to 1 to disable.
// The gate must stay larger than the biggest honest step between two frames.
const float SPIKE_GATE_DEG = 20.0f;
const float SPIKE_MATCH_DEG = 3.0f;
const int SPIKE_CONFIRM_FRAMES = 2;

// Low-pass on the motion after the speed and acceleration limits. Rounds off the
// stair-steps of a 20 Hz stream and the corners of each speed ramp. 0 turns it off.
const float SMOOTHING_TAU_MS = 150.0f;
const float SMOOTHING_SNAP_DEG = 0.05f;   // closer than this, land exactly on target

// true  = empty field releases the servo (no PWM pulse)
// false = empty field keeps the servo holding its last position
const bool EMPTY_MEANS_NO_SIGNAL = true;

const bool REQUIRE_CRC = true;

// A line that stops arriving this long before its newline is thrown away, never run.
const unsigned long LINE_TIMEOUT_MS = 100;
const int MAX_LINE_LENGTH = 64;

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
float smoothAngle[AXIS_COUNT];          // speed/acceleration profile toward targetAngle, before the low-pass
float velocity[AXIS_COUNT] = {0};       // deg/s of smoothAngle, for the acceleration limit
bool approaching[AXIS_COUNT] = {false}; // heading for the anti-backlash point past the target
bool needsApproach[AXIS_COUNT] = {false};  // last travelled against APPROACH_DIR; approach once settled
unsigned long targetChangedMs[AXIS_COUNT] = {0};

bool haveAccepted[AXIS_COUNT] = {false};   // false until the first frame after start/release
float candidateAngle[AXIS_COUNT];          // out-of-gate value waiting for confirmation
float holdCandidate[AXIS_COUNT] = {NAN, NAN, NAN, NAN, NAN, NAN};   // last in-deadband value, see HOLD_DEADBAND_DEG
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

// Neighbouring channels, four registers each, go out in one transaction; the PCA
// applies them all together at the STOP.
static bool pcaWriteChannels(uint8_t first, uint8_t count) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write((uint8_t)(REG_LED0_ON_L + 4 * first));
  for (uint8_t ch = first; ch < first + count; ch++) {
    uint16_t ticks = expectedTicks[ch];
    Wire.write((uint8_t)0);
    Wire.write((uint8_t)0);
    Wire.write((uint8_t)(ticks & 0xFF));
    Wire.write((uint8_t)(ticks >> 8));
  }
  return noteI2c(Wire.endTransmission() == 0);
}

static bool pcaWriteChannel(uint8_t channel) {
  return pcaWriteChannels(channel, 1);
}

static void pcaSetChannel(uint8_t channel, uint16_t ticks) {
  expectedTicks[channel] = ticks;
  pcaWriteChannel(channel);   // a failed write is put right by the next verify pass
}

// Arm A's two motors share one shaft. When their channels are neighbours both go out
// in one transaction, so the pair always changes together and a failed write can't
// leave one motor pulling against the other.
static void setArmA(uint16_t primaryTicks, uint16_t mirrorTicks) {
  uint8_t primary = pcaChannel(PCA_LABEL[ARM_A]);
  uint8_t mirror = pcaChannel(ARM_A_MIRROR_LABEL);
  expectedTicks[primary] = primaryTicks;
  expectedTicks[mirror] = mirrorTicks;
  if (mirror == primary + 1) {
    pcaWriteChannels(primary, 2);
  } else if (primary == mirror + 1) {
    pcaWriteChannels(mirror, 2);
  } else {
    pcaWriteChannel(primary);
    pcaWriteChannel(mirror);
  }
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
  if (axis == ARM_A) {
    float mirrored = 180.0f - angle + ARM_A_MIRROR_TRIM_DEG;
    setArmA(angleToTicks(angle), angleToTicks(mirrored));
    return;
  }
  pcaSetChannel(pcaChannel(PCA_LABEL[axis]), angleToTicks(angle));
}

// ---------- attach / release ----------

static void attachBase() {
  basePwm.attachPin(BASE_PIN, PWM_HZ, BASE_PWM_BITS);
}

// Forgets any speed the axis had: it starts from standstill.
static void resetMotion(int axis) {
  velocity[axis] = 0.0f;
  approaching[axis] = false;
  needsApproach[axis] = false;
}

// Standing still at its target, with nothing left to do.
static bool atRest(int axis) {
  return currentAngle[axis] == targetAngle[axis] && smoothAngle[axis] == targetAngle[axis] &&
         velocity[axis] == 0.0f && !approaching[axis] && !needsApproach[axis];
}

// Where the anti-backlash approach turns around: APPROACH_OVERSHOOT_DEG short of the
// target, seen from the approach direction. Kept inside the standard 0..180 range
// unless the target itself is outside it.
static float approachPoint(int axis) {
  float point = targetAngle[axis] - APPROACH_DIR[axis] * APPROACH_OVERSHOOT_DEG;
  float lo = fminf(targetAngle[axis], 0.0f);
  float hi = fmaxf(targetAngle[axis], 180.0f);
  return fminf(fmaxf(point, lo), hi);
}

static void moveServo(int axis, float angle) {
  if (!attached[axis] && !pendingAttach[axis]) {
    // Like the Uno firmware: the first command after release is where it starts
    // (we don't know where the unpowered servo was left). Later frames that arrive
    // before its turn in the queue only move the target, so they stay speed-limited.
    pendingAttach[axis] = true;
    resetMotion(axis);
    currentAngle[axis] = angle;
    smoothAngle[axis] = angle;
  }
  if (angle != targetAngle[axis]) targetChangedMs[axis] = millis();
  targetAngle[axis] = angle;
}

// Spike gate in front of moveServo; see SPIKE_GATE_DEG.
static void acceptTarget(int axis, float angle) {
  if (!haveAccepted[axis] || fabsf(angle - targetAngle[axis]) <= SPIKE_GATE_DEG) {
    if (candidateCount[axis] > 0) countSpike++;   // the held value was a one-off
    candidateCount[axis] = 0;
    haveAccepted[axis] = true;
    // Hold deadband: see HOLD_DEADBAND_DEG.
    if (attached[axis] && atRest(axis) && fabsf(angle - targetAngle[axis]) < HOLD_DEADBAND_DEG &&
        angle != holdCandidate[axis]) {
      holdCandidate[axis] = angle;
      return;
    }
    moveServo(axis, angle);
    return;
  }
  // Agreement: the held value again, or a value carrying on the same way from it. A
  // fast honest move (a slider drag) never repeats a value, but it keeps its direction;
  // a corrupted digit is a one-off that the next frame doesn't follow.
  float fromCandidate = angle - candidateAngle[axis];
  if (candidateCount[axis] > 0 &&
      (fabsf(fromCandidate) <= SPIKE_MATCH_DEG ||
       (fromCandidate * (candidateAngle[axis] - targetAngle[axis]) > 0.0f &&
        fabsf(fromCandidate) <= SPIKE_GATE_DEG))) {
    candidateAngle[axis] = angle;
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
    if (axis == ARM_A) setArmA(TICKS_FULL_OFF, TICKS_FULL_OFF);
    else pcaSetChannel(pcaChannel(PCA_LABEL[axis]), TICKS_FULL_OFF);
  }

  attached[axis] = false;
  resetMotion(axis);
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

// Each tick every axis runs a trapezoidal speed profile toward its target: speed changes
// by at most its acceleration limit, never exceeds its speed limit, and is always low
// enough to stop on the target. The low-pass then rounds off the profile's corners.
// A low-pass is a weighted average of past positions, so it can't overshoot or add
// acceleration: the limits still hold at the servo.
static void updateServoPositions(unsigned long now) {
  unsigned long dtMs = now - lastMotionMs;
  if (dtMs < MOTION_PERIOD_MS) return;
  lastMotionMs = now;
  if (dtMs > MAX_MOTION_DT_MS) dtMs = MAX_MOTION_DT_MS;
  float dt = dtMs / 1000.0f;
  float blend = (SMOOTHING_TAU_MS > 0.0f) ? 1.0f - expf(-(float)dtMs / SMOOTHING_TAU_MS) : 1.0f;

  for (int i = 0; i < AXIS_COUNT; i++) {
    if (!attached[i] || atRest(i)) continue;

    // Anti-backlash: an axis that got to its target travelling against APPROACH_DIR
    // waits until the target has settled, then goes past it and comes back once.
    int dir = APPROACH_DIR[i];
    if (dir != 0 && !approaching[i]) {
      if (velocity[i] * dir < 0.0f) needsApproach[i] = true;
      else if (velocity[i] * dir > 0.0f) needsApproach[i] = false;
      if (needsApproach[i] && velocity[i] == 0.0f && smoothAngle[i] == targetAngle[i] &&
          now - targetChangedMs[i] >= APPROACH_SETTLE_MS) {
        needsApproach[i] = false;
        approaching[i] = true;
      }
    }
    float goal = targetAngle[i];
    if (approaching[i]) {
      float point = approachPoint(i);
      // At or beyond the turning point and no longer moving the wrong way: the rest
      // of the move is in the approach direction.
      if ((point - smoothAngle[i]) * dir >= 0.0f && velocity[i] * dir >= 0.0f) approaching[i] = false;
      else goal = point;
    }

    // Profile, in smoothAngle.
    float accel = AXIS_MAX_ACCEL_DEG_PER_SEC2[i];
    float maxDv = accel * dt;
    float err = goal - smoothAngle[i];
    // Fastest speed that still stops exactly on the goal when it is cut by maxDv every
    // tick (the discrete form of sqrt(2 * accel * distance)).
    float stopTicks = 0.5f * (sqrtf(1.0f + 8.0f * fabsf(err) / (maxDv * dt)) - 1.0f);
    float want = fminf(stopTicks * maxDv, AXIS_MAX_SPEED_DEG_PER_SEC[i]);
    if (err < 0.0f) want = -want;
    // Stopping this tick is only allowed from a speed one tick's change can take away.
    bool canStop = fabsf(velocity[i]) <= maxDv;
    velocity[i] += fmaxf(-maxDv, fminf(maxDv, want - velocity[i]));
    float step = velocity[i] * dt;
    if (canStop && ((step * err > 0.0f && fabsf(step) >= fabsf(err)) || fabsf(err) < SMOOTHING_SNAP_DEG)) {
      smoothAngle[i] = goal;   // arrived
      velocity[i] = 0.0f;
    } else {
      smoothAngle[i] += step;
    }

    // Low-pass, in currentAngle.
    float before = currentAngle[i];
    currentAngle[i] += (smoothAngle[i] - currentAngle[i]) * blend;
    if (velocity[i] == 0.0f && fabsf(smoothAngle[i] - currentAngle[i]) < SMOOTHING_SNAP_DEG)
      currentAngle[i] = smoothAngle[i];

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

// ---------- parsing ----------

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
static bool processLine(const char *line, int len) {
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

  Serial.println("Ready (ESP32+PCA9685). Format: Gripper,WristYaw,WristPitch,ArmB,ArmA,Base,CRC8");
  Serial.println("Arm A -> PCA 1, mirrored on PCA 2 | Base -> ESP32 GPIO");
  Serial.println("Speed Control: per axis, with acceleration limit and anti-backlash approach");
}

void loop() {
  unsigned long now = millis();

  readSerial(now);
  serviceAttachQueue(now);
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
