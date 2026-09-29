#include <Servo.h>
#include <math.h>

// ================= CONFIGURATION =================
const int AXIS_COUNT = 6;  // 6 logical axes to control
const int SERVO_COUNT = 7; // 7 physical motors (Axis 5 uses 2 motors, plus 1 new motor)

// Pins 2, 3, 4, 5 for individual servos. Pins 6 & 7 for the mirrored pair. Pin 8 for the new axis.
const int servoPin[SERVO_COUNT] = {2, 3, 4, 5, 6, 7, 8};

// Logical axis -> servo index. Axis 5 (D8) must skip index 5 (pin 7),
// which is reserved for the mirrored Arm A motor.
const int axisServo[AXIS_COUNT] = {0, 1, 2, 3, 4, 6};
const int MIRROR_SERVO = 5; // Pin 7, mirrors axis 4 (Pin 6)

// ⚠️ MG996R CORRECTED PULSE WIDTH RANGE ⚠️
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// === SPEED CONTROL CONFIGURATION ===
const float MAX_SPEED_DEG_PER_SEC = 60.0f; 
// ===================================

// true  = empty field detaches the servo, so no PWM signal is sent.
// false = empty field keeps the servo attached at its last position.
const bool EMPTY_MEANS_NO_SIGNAL = true;

// Optional timeout so commands without newline can still work.
const unsigned long LINE_TIMEOUT_MS = 30;
const int MAX_LINE_LENGTH = 64;

Servo servo[SERVO_COUNT];
bool attached[AXIS_COUNT] = {false};

// Speed control state tracking (tracks the 6 logical axes)
float currentAngle[AXIS_COUNT] = {0};
int targetAngle[AXIS_COUNT] = {0};
unsigned long lastMoveTime[AXIS_COUNT] = {0};

String inputLine = "";
unsigned long lastCharTime = 0;

void setup() {
  Serial.begin(9600);

  // Known no-PWM state on every physical servo pin (2..8)
  for (int s = 0; s < SERVO_COUNT; s++) {
    pinMode(servoPin[s], OUTPUT);
    digitalWrite(servoPin[s], LOW);
  }

  // Initialize the 6 logical axes
  for (int i = 0; i < AXIS_COUNT; i++) {
    currentAngle[i] = 90.0f; // Assume center start to prevent initial jump
    targetAngle[i] = 90;
  }

  Serial.println(F("Ready. Format: D2,D3,D4,D5,D6(mirrored),D8"));
  Serial.println(F("(Note: 5th value controls Pin 6 and mirrors to Pin 7)"));
  Serial.print(F("Speed Control: "));
  Serial.print(MAX_SPEED_DEG_PER_SEC);
  Serial.println(F(" deg/sec"));
}

void loop() {
  // --- 1. Handle Serial Input ---
  while (Serial.available()) {
    char c = Serial.read();
    lastCharTime = millis();

    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        processCommand(inputLine);
      }
      inputLine = "";
    } else {
      if (inputLine.length() < MAX_LINE_LENGTH) {
        inputLine += c;
      }
    }
  }

  if (inputLine.length() > 0 && millis() - lastCharTime >= LINE_TIMEOUT_MS) {
    processCommand(inputLine);
    inputLine = "";
  }

  // --- 2. Non-Blocking Speed Control Loop ---
  updateServoPositions();
}

/**
 * Interpolates servo positions toward their targets based on MAX_SPEED_DEG_PER_SEC.
 */
void updateServoPositions() {
  unsigned long now = millis();

  for (int i = 0; i < AXIS_COUNT; i++) {
    if (!attached[i]) continue;
    if (currentAngle[i] == (float)targetAngle[i]) continue;

    // Calculate time delta since last update for this axis
    unsigned long dt = now - lastMoveTime[i];
    if (dt < 1) continue; 

    float maxStep = MAX_SPEED_DEG_PER_SEC * ((float)dt / 1000.0f);
    float diff = (float)targetAngle[i] - currentAngle[i];

    if (fabs(diff) <= maxStep) {
      currentAngle[i] = (float)targetAngle[i];
    } else {
      currentAngle[i] += (diff > 0) ? maxStep : -maxStep;
    }

    int angle = (int)round(currentAngle[i]);
    
    // Write to master motor
    servo[axisServo[i]].write(angle);
    
    // If this is the 5th axis (index 4), mirror the angle for the opposing motor on Pin 7
    if (i == 4) {
      int mirroredAngle = 180 - angle; 
      servo[MIRROR_SERVO].write(mirroredAngle);
    }
    
    lastMoveTime[i] = now;
  }
}

void processCommand(const String &line) {
  int start = 0;

  // Expecting 6 comma-separated values now
  for (int i = 0; i < AXIS_COUNT; i++) {
    String field;
    int commaIndex = line.indexOf(',', start);

    if (commaIndex >= 0) {
      field = line.substring(start, commaIndex);
      start = commaIndex + 1;
    } else {
      field = line.substring(start);
      start = line.length();
    }

    field.trim();

    if (field.length() == 0) {
      if (EMPTY_MEANS_NO_SIGNAL) {
        releaseServo(i);
      }
    } else {
      long value;
      if (parseInteger(field, value)) {
        if (value >= 0 && value <= 180) {
          moveServo(i, (int)value);
        }
      }
    }

    if (commaIndex < 0) break;
  }
}

void moveServo(int index, int angle) {
  if (!attached[index]) {
    noInterrupts();
    servo[axisServo[index]].attach(servoPin[axisServo[index]], SERVO_MIN_US, SERVO_MAX_US);
    
    // If attaching the 5th axis, attach the mirrored motor as well
    if (index == 4) {
      servo[MIRROR_SERVO].attach(servoPin[MIRROR_SERVO], SERVO_MIN_US, SERVO_MAX_US);
    }
    
    interrupts();
    attached[index] = true;
    
    currentAngle[index] = (float)angle;
    servo[axisServo[index]].write(angle);
    
    if (index == 4) {
      servo[MIRROR_SERVO].write(180 - angle);
    }
    
    lastMoveTime[index] = millis();
  }
  
  targetAngle[index] = angle;
}

void releaseServo(int index) {
  if (attached[index]) {
    servo[axisServo[index]].detach();
    pinMode(servoPin[axisServo[index]], OUTPUT);
    digitalWrite(servoPin[axisServo[index]], LOW);
    
    // If releasing the 5th axis, detach the mirrored motor as well
    if (index == 4) {
      servo[MIRROR_SERVO].detach();
      pinMode(servoPin[MIRROR_SERVO], OUTPUT);
      digitalWrite(servoPin[MIRROR_SERVO], LOW);
    }
    
    attached[index] = false;
    
    // Reset tracking state
    currentAngle[index] = 90.0f;
    targetAngle[index] = 90;
  }
}

bool parseInteger(const String &s, long &result) {
  if (s.length() == 0) return false;

  int i = 0;
  bool negative = false;

  if (s[0] == '-') {
    negative = true;
    i = 1;
  } else if (s[0] == '+') {
    i = 1;
  }

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