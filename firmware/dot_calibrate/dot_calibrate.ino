#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// =============================
// DOT SERVO SETUP
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
const int INVERTED_SERVOS[] = {0, 2, 4, 6, 9, 11};
const int INVERTED_SERVO_COUNT = sizeof(INVERTED_SERVOS) / sizeof(INVERTED_SERVOS[0]);

// Servo groups
const int HIP_SERVOS[]  = {0, 1, 2, 3};
const int LEG_SERVOS[]  = {4, 5, 6, 7};
const int FEET_SERVOS[] = {8, 9, 10, 11};

// Smoothing settings
const int STEP_SIZE = 2;
const int UPDATE_INTERVAL = 15;

int currentAngle[NUM_SERVOS];
int targetAngle[NUM_SERVOS];

unsigned long lastUpdate = 0;

// Offset for each servo to fix mechanical alignment
int servoOffset[NUM_SERVOS] = {
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0
};

bool isInverted(int servo) {
  for (int i = 0; i < INVERTED_SERVO_COUNT; i++) {
    if (INVERTED_SERVOS[i] == servo) {
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
  logicalAngle += servoOffset[servo];
  logicalAngle = constrain(logicalAngle, 0, 180);

  int actualAngle = logicalAngle;

  if (isInverted(servo)) {
    actualAngle = 180 - logicalAngle;
  }

  int pulse = angleToPulse(actualAngle);
  pca.setPWM(servo, 0, pulse);
}

void setServoTarget(int servo, int angle) {
  if (servo < 0 || servo >= NUM_SERVOS) return;
  targetAngle[servo] = constrain(angle, 0, 180);
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

void moveServoGroup(const int servoList[], int servoCount, int angle) {
  for (int i = 0; i < servoCount; i++) {
    setServoTarget(servoList[i], angle);
  }
}

void moveAllHipServos(int angle) {
  moveServoGroup(HIP_SERVOS, 4, angle);
}

void moveAllLegServos(int angle) {
  moveServoGroup(LEG_SERVOS, 4, angle);
}

void moveAllFeetServos(int angle) {
  moveServoGroup(FEET_SERVOS, 4, angle);
}

void standPose() {
  moveAllHipServos(110);
  moveAllLegServos(40);
  moveAllFeetServos(90);
}

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

  Serial.println("Commands:");
  Serial.println("0 -> hips 45");
  Serial.println("1 -> hips 90");
  Serial.println("2 -> hips 135");
  Serial.println("3 -> legs 45");
  Serial.println("4 -> legs 90");
  Serial.println("5 -> legs 135");
  Serial.println("6 -> feet 45");
  Serial.println("7 -> feet 90");
  Serial.println("8 -> feet 135");
  Serial.println("9 -> stand pose");
}

void loop() {
  updateServosSmooth();

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int command = input.toInt();
    int hip_angle = -1;
    int leg_angle = -1;
    int feet_angle = -1;

    if (command == 0) {
      hip_angle = 45;
    }
    else if (command == 1) {
      hip_angle = 90;
    }
    else if (command == 2) {
      hip_angle = 135;
    }
    else if (command == 3) {
      leg_angle = 45;
    }
    else if (command == 4) {
      leg_angle = 90;
    }
    else if (command == 5) {
      leg_angle = 135;
    }
    else if (command == 6) {
      feet_angle = 45;
    }
    else if (command == 7) {
      feet_angle = 90;
    }
    else if (command == 8) {
      feet_angle = 135;
    }
    else if (command == 9) {
      standPose();
    }

    if (hip_angle != -1) {
      moveAllHipServos(hip_angle);
    }

    if (leg_angle != -1) {
      moveAllLegServos(leg_angle);
    }

    if (feet_angle != -1) {
      moveAllFeetServos(feet_angle);
    }
  }
}