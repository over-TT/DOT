#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// =============================
// DOT SERVO + IK SETUP
// =============================

// ESP32 I2C pins connected to the PCA9685
const int SDA_PIN = 34;
const int SCL_PIN = 33;

// PCA9685 pulse range for the servos
const int SERVO_MIN = 100;
const int SERVO_MAX = 520;

const int NUM_SERVOS = 12;

// Servos that are mounted backwards
// Add servo ports here if they move in the wrong direction
const int REVERSED_SERVOS[] = {0, 2, 4, 6, 9, 11};
const int REVERSED_SERVO_COUNT = sizeof(REVERSED_SERVOS) / sizeof(REVERSED_SERVOS[0]);

// Smoothing settings
const int STEP_SIZE = 1;
const int UPDATE_INTERVAL = 12;

// Turn this on only if you want the Serial Monitor to become a waterfall of numbers
const bool PRINT_IK_DEBUG = false;

// Leg servo groups: {hip side, upper leg, foot/knee}
const int BACK_RIGHT[3]  = {3, 7, 11};
const int FRONT_RIGHT[3] = {1, 5, 9};
const int BACK_LEFT[3]   = {2, 6, 10};
const int FRONT_LEFT[3]  = {0, 4, 8};

// Offset for each servo to fix mechanical alignment
int servoOffset[NUM_SERVOS] = {
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0
};

// Extra geometry offsets for upper leg and knee tuning.
// Applied to leg[1] (upper) and leg[2] (knee/foot) in moveLegToXYZ().
int legGeometryOffset[NUM_SERVOS] = {
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0
};

int currentAngle[NUM_SERVOS];
int targetAngle[NUM_SERVOS];

unsigned long lastUpdate = 0;

// =============================
// IK MATH
// =============================

struct IKResult {
  float g;      // side hip angle
  float a;      // upper leg angle
  float t;      // knee angle
  bool valid;   // false if point cannot be reached
};

IKResult ik(float x, float y, float z, bool left) {
  IKResult result;

  result.g = 0;
  result.a = 0;
  result.t = 0;
  result.valid = false;

  // Robot dimensions
  const float o  = -0.6f;  // hip offset
  const float l1 = 5.0f;   // upper leg length
  const float l2 = 5.0f;   // lower leg length

  // Mirror Y for the left side so a positive Y always means "outward from the body"
  float yLocal = left ? -y : y;

  // Distance in the Y-Z view
  float r1 = sqrtf(yLocal * yLocal + z * z);

  // Avoid divide-by-zero
  if (r1 < 0.0001f) {
    return result;
  }

  // First side angle
  float c1 = o / r1;
  if (c1 > 1.0f) c1 = 1.0f;
  if (c1 < -1.0f) c1 = -1.0f;

  float P = atan2f(yLocal, z);
  float g = P + acosf(c1);

  // Rotate target into the 2D plane of the leg
  float x2 = x;
  float z2 = -yLocal * cosf(g) + z * sinf(g);

  // Distance from hip joint to target in the 2D leg plane
  float d = sqrtf(x2 * x2 + z2 * z2);

  // Check if target is reachable
  if (d > l1 + l2) {
    return result;
  }

  if (d < fabsf(l1 - l2)) {
    return result;
  }

  // Solve the 2-link leg triangle
  float c2 = (l1 * l1 + l2 * l2 - d * d) / (2.0f * l1 * l2);
  if (c2 > 1.0f) c2 = 1.0f;
  if (c2 < -1.0f) c2 = -1.0f;

  float kneeInner = acosf(c2);
  float t = PI - (kneeInner - PI / 2);

  float c3 = (l1 * l1 + d * d - l2 * l2) / (2.0f * l1 * d);
  if (c3 > 1.0f) c3 = 1.0f;
  if (c3 < -1.0f) c3 = -1.0f;

  float beta = acosf(c3);
  float p = atan2f(z2, x2);
  float a = PI - p - beta;

  // Convert radians to degrees
  result.g = g * 180.0f / PI;
  result.a = a * 180.0f / PI;
  result.t = t * 180.0f / PI;
  result.valid = true;

  return result;
}

// =============================
// SERVO CONTROL
// =============================

bool isReversed(int servo) {
  for (int i = 0; i < REVERSED_SERVO_COUNT; i++) {
    if (REVERSED_SERVOS[i] == servo) {
      return true;
    }
  }

  return false;
}

int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void writeServoLogical(int servo, int logicalAngle) {
  if (servo < 0 || servo >= NUM_SERVOS) return;

  logicalAngle += servoOffset[servo];
  logicalAngle = constrain(logicalAngle, 0, 180);

  int actualAngle = logicalAngle;

  if (isReversed(servo)) {
    actualAngle = 180 - logicalAngle;
  }

  int pulse = angleToPulse(actualAngle);
  pca.setPWM(servo, 0, pulse);
}

void setServoTarget(int servo, int angle) {
  if (servo < 0 || servo >= NUM_SERVOS) return;

  targetAngle[servo] = constrain(angle, 45, 135);
}

void updateServosSmooth() {
  if (millis() - lastUpdate < UPDATE_INTERVAL) return;

  lastUpdate = millis();

  for (int servo = 0; servo < NUM_SERVOS; servo++) {
    if (currentAngle[servo] < targetAngle[servo]) {
      currentAngle[servo] += STEP_SIZE;

      if (currentAngle[servo] > targetAngle[servo]) {
        currentAngle[servo] = targetAngle[servo];
      }

      writeServoLogical(servo, currentAngle[servo]);
    }
    else if (currentAngle[servo] > targetAngle[servo]) {
      currentAngle[servo] -= STEP_SIZE;

      if (currentAngle[servo] < targetAngle[servo]) {
        currentAngle[servo] = targetAngle[servo];
      }

      writeServoLogical(servo, currentAngle[servo]);
    }
  }
}

// =============================
// BASIC SERVO GROUP MOVES
// =============================

void moveAllHipServos(int angle) {
  setServoTarget(0, angle);
  setServoTarget(1, angle);
  setServoTarget(2, angle);
  setServoTarget(3, angle);
}

void moveAllLegServos(int angle) {
  setServoTarget(4, angle);
  setServoTarget(5, angle);
  setServoTarget(6, angle);
  setServoTarget(7, angle);
}

void moveAllFeetServos(int angle) {
  setServoTarget(8, angle);
  setServoTarget(9, angle);
  setServoTarget(10, angle);
  setServoTarget(11, angle);
}

void standPose() {
  moveAllHipServos(110);
  moveAllLegServos(45);
  moveAllFeetServos(90);
}

// =============================
// MOVE ONE LEG USING IK
// =============================

void moveLegToXYZ(const int leg[3], float x, float y, float z, bool left) {
  IKResult angles = ik(x, y, z, left);

  if (!angles.valid) {
    Serial.print("IK failed for x=");
    Serial.print(x);
    Serial.print(" y=");
    Serial.print(y);
    Serial.print(" z=");
    Serial.println(z);
    return;
  }

  int hipTarget  = (int)angles.g;
  int legTarget  = (int)angles.a + legGeometryOffset[leg[1]];
  int footTarget = (int)angles.t + legGeometryOffset[leg[2]];

  if (PRINT_IK_DEBUG) {
    Serial.print("RAW IK -> g=");
    Serial.print(angles.g);
    Serial.print("  a=");
    Serial.print(angles.a);
    Serial.print("  t=");
    Serial.println(angles.t);

    Serial.print("TARGETS -> hip=");
    Serial.print(hipTarget);
    Serial.print("  leg=");
    Serial.print(legTarget);
    Serial.print("  foot=");
    Serial.println(footTarget);
  }

  setServoTarget(leg[0], hipTarget);
  setServoTarget(leg[1], legTarget);
  setServoTarget(leg[2], footTarget);
}

// =============================
// SETUP
// =============================

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  pca.begin();
  pca.setPWMFreq(50);

  delay(500);

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i] = 90;
    targetAngle[i] = 90;
    writeServoLogical(i, 90);
  }

  Serial.println();
  Serial.println("=== DOT SERVO / IK CONTROL ===");
  Serial.println();
  Serial.println("Basic test commands:");
  Serial.println("  0  -> all hips to 45");
  Serial.println("  1  -> all hips to 90");
  Serial.println("  2  -> all hips to 135");
  Serial.println("  3  -> all upper-leg servos to 45");
  Serial.println("  4  -> all upper-leg servos to 90");
  Serial.println("  5  -> all upper-leg servos to 135");
  Serial.println("  6  -> all foot/knee servos to 45");
  Serial.println("  7  -> all foot/knee servos to 90");
  Serial.println("  8  -> all foot/knee servos to 135");
  Serial.println("  9  -> stand pose");
  Serial.println();
  Serial.println("IK command:");
  Serial.println("  ik <leg> <x> <y> <z>");
  Serial.println();
  Serial.println("Leg numbers:");
  Serial.println("  0 = front left");
  Serial.println("  1 = front right");
  Serial.println("  2 = back left");
  Serial.println("  3 = back right");
  Serial.println();
  Serial.println("Example:");
  Serial.println("  ik 0 4.0 1.0 6.0");
  Serial.println();
}

// =============================
// MAIN LOOP
// =============================

void loop() {
  updateServosSmooth();

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int legNumber;
    float x, y, z;

    if (sscanf(input.c_str(), "ik %d %f %f %f", &legNumber, &x, &y, &z) == 4) {
      const int* chosenLeg = nullptr;
      bool isLeft = false;
      const char* legName = "";

      if (legNumber == 0) {
        chosenLeg = FRONT_LEFT;
        isLeft = true;
        legName = "front left";
      }
      else if (legNumber == 1) {
        chosenLeg = FRONT_RIGHT;
        isLeft = false;
        legName = "front right";
      }
      else if (legNumber == 2) {
        chosenLeg = BACK_LEFT;
        isLeft = true;
        legName = "back left";
      }
      else if (legNumber == 3) {
        chosenLeg = BACK_RIGHT;
        isLeft = false;
        legName = "back right";
      }

      if (chosenLeg != nullptr) {
        Serial.print("Moving ");
        Serial.print(legName);
        Serial.print(" to x=");
        Serial.print(x);
        Serial.print(" y=");
        Serial.print(y);
        Serial.print(" z=");
        Serial.println(z);

        moveLegToXYZ(chosenLeg, x, y, z, isLeft);
      }
      else {
        Serial.println("Bad leg number. Use 0, 1, 2, or 3.");
      }
    }
    else {
      int command = input.toInt();

      if (command == 0) {
        moveAllHipServos(45);
      }
      else if (command == 1) {
        moveAllHipServos(90);
      }
      else if (command == 2) {
        moveAllHipServos(135);
      }
      else if (command == 3) {
        moveAllLegServos(45);
      }
      else if (command == 4) {
        moveAllLegServos(90);
      }
      else if (command == 5) {
        moveAllLegServos(135);
      }
      else if (command == 6) {
        moveAllFeetServos(45);
      }
      else if (command == 7) {
        moveAllFeetServos(90);
      }
      else if (command == 8) {
        moveAllFeetServos(135);
      }
      else if (command == 9) {
        standPose();
      }
    }
  }
}