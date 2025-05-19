#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

// --- Pin Definitions ---
#define MOTOR_INTERFACE_TYPE 1
#define M1_STEP_PIN 2
#define M1_DIR_PIN  5
#define M2_STEP_PIN 3
#define M2_DIR_PIN  6
#define ENABLE_PIN 8
#define PEN_CONTROL_PIN 4
#define PEN_DOWN_STATE HIGH
#define PEN_UP_STATE   LOW

// --- Robot Geometry ---
const double L1 = 150.0;
const double L2 = 150.0;

// --- Interpolation Settings ---
const double LINE_SEGMENT_LENGTH = 1.0;
const int CIRCLE_SEGMENTS = 72;

// --- Stepper Configuration ---
const int STEPS_PER_REV_MOTOR = 200;
const int MICROSTEPS = 16;
const int GEAR_RATIO = 3;
const long STEPS_PER_REV_JOINT = (long)STEPS_PER_REV_MOTOR * MICROSTEPS * GEAR_RATIO;
const double STEPS_PER_RADIAN = (double)STEPS_PER_REV_JOINT / (2.0 * PI);

// Stepper Instances
AccelStepper stepper1(MOTOR_INTERFACE_TYPE, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, M2_STEP_PIN, M2_DIR_PIN);
MultiStepper steppers;
long currentPositions[2];

// Structure for Joint Angles
struct JointAngles {
  double theta1;
  double theta2;
  bool reachable;
};

// Serial Input Variables
String inputString = "";
boolean stringComplete = false;
enum InputState {
  WAITING_FOR_COMMAND,
  WAITING_FOR_X0,
  WAITING_FOR_Y0,
  WAITING_FOR_X1,
  WAITING_FOR_Y1,
  WAITING_FOR_RADIUS,
  WAITING_FOR_CENTER_X,
  WAITING_FOR_CENTER_Y
};
InputState currentState = WAITING_FOR_COMMAND;
double x0, y0, x1, y1, circleRadius, circleCenterX, circleCenterY;

// --- IK Function ---
JointAngles calculateIK(double targetX, double targetY) {
  JointAngles angles = {0.0, 0.0, false};
  double distanceSq = targetX * targetX + targetY * targetY;
  double distance = sqrt(distanceSq);

  if (distance > L1 + L2 + 0.01 || distance < fabs(L1 - L2) - 0.01 || distance < 0.01) {
    return angles;
  }

  double cosTheta2 = (distanceSq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  cosTheta2 = constrain(cosTheta2, -1.0, 1.0);
  angles.theta2 = acos(cosTheta2);

  double alpha = atan2(targetY, targetX);
  double cosBeta = (L1 * L1 + distanceSq - L2 * L2) / (2.0 * L1 * distance);
  cosBeta = constrain(cosBeta, -1.0, 1.0);
  double beta = acos(cosBeta);
  angles.theta1 = alpha - beta;
  angles.reachable = true;
  return angles;
}

// --- Angle to Steps ---
long angleToSteps(double angleRadians) {
  return (long)(angleRadians * STEPS_PER_RADIAN);
}

// --- Pen Control ---
void penSetup() {
  pinMode(PEN_CONTROL_PIN, OUTPUT);
  penUp();
}

void penUp() {
  digitalWrite(PEN_CONTROL_PIN, PEN_UP_STATE);
  delay(200);
}

void penDown() {
  digitalWrite(PEN_CONTROL_PIN, PEN_DOWN_STATE);
  delay(200);
}

// --- Blocking Move ---
void moveToXY_blocking(double targetX, double targetY) {
  JointAngles targetAngles = calculateIK(targetX, targetY);
  if (targetAngles.reachable) {
    long targetSteps1 = angleToSteps(targetAngles.theta1);
    long targetSteps2 = angleToSteps(-1 * targetAngles.theta2);
    currentPositions[0] = targetSteps1;
    currentPositions[1] = targetSteps2;
    steppers.moveTo(currentPositions);
    while (steppers.run());
  } else {
    Serial.println("  Error: Blocking move target UNREACHABLE!");
  }
}

// --- Draw Line ---
void drawLine(double x0, double y0, double x1, double y1) {
  double deltaX = x1 - x0;
  double deltaY = y1 - y0;
  double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
  int numSegments = max(1, (int)(distance / LINE_SEGMENT_LENGTH));

  for (int i = 1; i <= numSegments; i++) {
    double t = (double)i / numSegments;
    double currentX = x0 + deltaX * t;
    double currentY = y0 + deltaY * t;
    JointAngles currentAngles = calculateIK(currentX, currentY);

    if (currentAngles.reachable) {
      long steps1 = angleToSteps(currentAngles.theta1);
      long steps2 = angleToSteps(-1 * currentAngles.theta2);
      currentPositions[0] = steps1;
      currentPositions[1] = steps2;
      steppers.moveTo(currentPositions);
      while (steppers.run());
    } else {
      Serial.print("  Unreachable point during line at (");
      Serial.print(currentX, 2); Serial.print(", "); Serial.print(currentY, 2); Serial.println(")");
      break;
    }
  }
}

// --- Draw Circle ---
void drawCircle(double centerX, double centerY, double radius) {
  double perimeter = 2.0 * PI * radius;
  int numSegments = max(CIRCLE_SEGMENTS, (int)(perimeter / LINE_SEGMENT_LENGTH));

  moveToXY_blocking(centerX + radius, centerY);
  penDown();

  for (int i = 1; i <= numSegments; i++) {
    double angle = (2.0 * PI * i) / numSegments;
    double x = centerX + radius * cos(angle);
    double y = centerY + radius * sin(angle);
    JointAngles angles = calculateIK(x, y);

    if (angles.reachable) {
      currentPositions[0] = angleToSteps(angles.theta1);
      currentPositions[1] = angleToSteps(-1 * angles.theta2);
      steppers.moveTo(currentPositions);
      while (steppers.run());
    } else {
      Serial.print("  Unreachable point during circle at (");
      Serial.print(x, 2); Serial.print(", "); Serial.print(y, 2); Serial.println(")");
      break;
    }
  }

  penUp();
}

// --- Help Message ---
void printHelp() {
  Serial.println("Available commands:");
  Serial.println("  move     - Move pen to position without drawing");
  Serial.println("  line     - Draw line from point A to B");
  Serial.println("  circle   - Draw circle at center with radius");
  Serial.println("  pen up   - Raise the pen");
  Serial.println("  pen down - Lower the pen");
  Serial.println("  cancel   - Cancel current operation");
}

// --- Serial Input Processing ---
void processSerialInput() {
  if (!stringComplete) return;
  inputString.trim();

  if (inputString.equalsIgnoreCase("cancel")) {
    currentState = WAITING_FOR_COMMAND;
    Serial.println("Operation cancelled.");
    stringComplete = false;
    return;
  }

  switch (currentState) {
    case WAITING_FOR_COMMAND:
      if (inputString.equalsIgnoreCase("line")) {
        currentState = WAITING_FOR_X0;
        Serial.println("Enter X0:");
      } else if (inputString.equalsIgnoreCase("move")) {
        currentState = WAITING_FOR_X0;
        Serial.println("Enter X:");
      } else if (inputString.equalsIgnoreCase("circle")) {
        currentState = WAITING_FOR_CENTER_X;
        Serial.println("Enter center X:");
      } else if (inputString.equalsIgnoreCase("pen up") || inputString.equalsIgnoreCase("up")) {
        penUp();
        Serial.println("Pen is now UP");
      } else if (inputString.equalsIgnoreCase("pen down") || inputString.equalsIgnoreCase("down")) {
        penDown();
        Serial.println("Pen is now DOWN");
      } else if (inputString.equalsIgnoreCase("help")) {
        printHelp();
      } else {
        Serial.println("Unknown command. Type 'help' for a list.");
      }
      break;

    case WAITING_FOR_X0:
      x0 = inputString.toFloat();
      Serial.println("Enter Y0:");
      currentState = WAITING_FOR_Y0;
      break;

    case WAITING_FOR_Y0:
      y0 = inputString.toFloat();
      if (inputString.equalsIgnoreCase("move")) {
        moveToXY_blocking(x0, y0);
        Serial.println("Move complete.");
        currentState = WAITING_FOR_COMMAND;
      } else {
        Serial.println("Enter X1:");
        currentState = WAITING_FOR_X1;
      }
      break;

    case WAITING_FOR_X1:
      x1 = inputString.toFloat();
      Serial.println("Enter Y1:");
      currentState = WAITING_FOR_Y1;
      break;

    case WAITING_FOR_Y1:
      y1 = inputString.toFloat();
      moveToXY_blocking(x0, y0);
      drawLine(x0, y0, x1, y1);
      Serial.println("Line drawing complete.");
      currentState = WAITING_FOR_COMMAND;
      break;

    case WAITING_FOR_CENTER_X:
      circleCenterX = inputString.toFloat();
      Serial.println("Enter center Y:");
      currentState = WAITING_FOR_CENTER_Y;
      break;

    case WAITING_FOR_CENTER_Y:
      circleCenterY = inputString.toFloat();
      Serial.println("Enter radius:");
      currentState = WAITING_FOR_RADIUS;
      break;

    case WAITING_FOR_RADIUS:
      circleRadius = inputString.toFloat();
      if (circleRadius <= 0) {
        Serial.println("Radius must be > 0");
      } else {
        drawCircle(circleCenterX, circleCenterY, circleRadius);
        Serial.println("Circle drawing complete.");
        currentState = WAITING_FOR_COMMAND;
      }
      break;
  }

  stringComplete = false;
}

// --- Serial Event ---
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') stringComplete = true;
  }
}

// --- Setup ---
void setup() {
  Serial.begin(9600);
  penSetup();
  stepper1.setMaxSpeed(1000.0);
  stepper2.setMaxSpeed(1000.0);
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  Serial.println("Ready. Type 'help' for commands.");
}

// --- Main Loop ---
void loop() {
  processSerialInput();
}
