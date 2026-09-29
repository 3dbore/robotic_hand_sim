#include <Servo.h>
#include <math.h>

// ================= CONFIGURATION =================
const int AXIS_COUNT = 5;  // 5 logical axes to control
const int SERVO_COUNT = 6; // 6 physical motors (Axis 5 uses 2 motors)

// Pins 2, 3, 4, 5 for individual servos. Pins 6 & 7 for the mirrored pair.
const int servoPin[SERVO_COUNT] = {2, 3, 4, 5, 6, 7};

// ⚠️ MG996R CORRECTED PULSE WIDTH RANGE ⚠️
// 500-2500µs is recommended for MG996R to reach full 0-180° travel without stalling.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// === SPEED CONTROL CONFIGURATION ===
// Maximum speed in degrees per second. 
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

// Speed control state tracking (tracks the 5 logical axes)
float currentAngle[AXIS_COUNT] = {0};
int targetAngle[AXIS_COUNT] = {0};
unsigned long lastMoveTime[AXIS_COUNT] = {0};

String inputLine = "";
unsigned long lastCharTime = 0;

void setup() {
  Serial.begin(9600);

  // Known no-PWM state at startup.
  for (int i = 0; i < AXIS_COUNT; i++) {
    pinMode(servoPin[i], OUTPUT);
    digitalWrite(servoPin[i], LOW);
    currentAngle[i] = 90.0f; // Assume center start to prevent initial jump
    targetAngle[i] = 90;
  }
  
  // Initialize the mirrored slave motor pin (Pin 7)
  pinMode(servoPin[5], OUTPUT);
  digitalWrite(servoPin[5], LOW);

  Serial.println(F("Ready. Format: angleD2,angleD3,angleD4,angleD5,angleD6"));
  Serial.println(F("(Note: angleD6 controls both Pin 6 and mirrors to Pin 7)"));
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
    servo[i].write(angle);
    
    // If this is the 5th axis (index 4), mirror the angle for the opposing motor on Pin 7
    if (i == 4) {
      int mirroredAngle = 180 - angle; 
      servo[5].write(mirroredAngle);
    }
    
    lastMoveTime[i] = now;
  }
}

void processCommand(const String &line) {
  int start = 0;

  // Expecting 5 comma-separated values now
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
    servo[index].attach(servoPin[index], SERVO_MIN_US, SERVO_MAX_US);
    
    // If attaching the 5th axis, attach the mirrored motor as well
    if (index == 4) {
      servo[5].attach(servoPin[5], SERVO_MIN_US, SERVO_MAX_US);
    }
    
    interrupts();
    attached[index] = true;
    
    currentAngle[index] = (float)angle;
    servo[index].write(angle);
    
    if (index == 4) {
      servo[5].write(180 - angle);
    }
    
    lastMoveTime[index] = millis();
  }
  
  targetAngle[index] = angle;
}

void releaseServo(int index) {
  if (attached[index]) {
    servo[index].detach();
    pinMode(servoPin[index], OUTPUT);
    digitalWrite(servoPin[index], LOW);
    
    // If releasing the 5th axis, detach the mirrored motor as well
    if (index == 4) {
      servo[5].detach();
      pinMode(servoPin[5], OUTPUT);
      digitalWrite(servoPin[5], LOW);
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