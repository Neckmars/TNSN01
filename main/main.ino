#include <Servo.h>
#include <EEPROM.h>

#define LT_L3_PIN A5
#define LT_L2_PIN A4
#define LT_L1_PIN A3
#define LT_R1_PIN A2
// Middle of line
#define LT_R2_PIN A1
#define LT_R3_PIN A0

#define MOTORLEFT1 5
#define MOTORLEFT2 6
#define MOTORRIGHT1 3
#define MOTORRIGHT2 11

#define ENCODER_RIGHT_A 4
#define ENCODER_RIGHT_B 2
#define ENCODER_LEFT_A 0
#define ENCODER_LEFT_B 1

#define FULL_TURN 3840
#define TURNRATIO 0.65
#define ON_LINE_FACTOR .9
#define ALMOST_ON_LINE_FACTOR .80
#define OFF_LINE_FACTOR .7

#define TRIGPIN 8
#define ECHOPIN 7

#define GRIPPERPIN 9
#define VERTICALPIN 10

#define GRIPPER_CLOSED_POS 20
#define GRIPPER_OPEN_POS 90
#define GRIPPER_DROPOFF_POS 35
#define VERTICAL_DOWN_POS 175
#define VERTICAL_PRE_OPEN_POS 105
#define VERTICAL_UP_POS 60
#define VERTICAL_DRIVE_POS 85
#define DISTANCE_FROM_GRIPPER 10

#define STARTBUTTON 12

#define ON_LINE 800  //Value for being on the line

#define DEBUG_PID 0
#define DEBUG_LINE_SENSORS 0
#define DEBUG_TURN_SENSORS 0
#define DEBUG_DISTANCE_SENSORS 0
#define DEBUG_TURNING 1
#define DEBUG_MOTOR_SPEED 0
#define DEBUG_FLAGS 1
#define DO_CALIBRATION 0

Servo gripperServo;
Servo verticalServo;

// In order that would represent physical location, needed to calculate weights based on horizontal position
const int sensorPins[6] = {
  LT_L3_PIN, LT_L2_PIN, LT_L1_PIN,
  LT_R1_PIN, LT_R2_PIN, LT_R3_PIN
};

const int weightArray[6] = {
  1500, 1000, 500, -500, -1000, -1500
};

// Important that these are int, not bool
int searchingForIntersection = 0;
int hasLeftTurn = 0;
int hasRightTurn = 0;
int hasForwardTurn = 0;

int lastError;
int integral;

int sensorOnLine[6] = { 0, 0, 0, 0, 0, 0 };
int sensorAlmostOnLine[6] = { 0, 0, 0, 0, 0, 0 };
int sensorOffLine[6] = { 0, 0, 0, 0, 0, 0 };

struct CalibrationValues {
  int sensorOnLine[6];
  int sensorAlmostOnLine[6];
  int sensorOffLine[6];
};

float Kp = 0.25;
float Ki = 0.00024;
float Kd = 8;

float amountOfPIDS = 0;

const int MAX_CORRECTION = 55;

const int baseSpeed = 200;

volatile long encoderRight = 0;
volatile long encoderLeft = 0;

long lastTime = millis();
long startTime = millis();
long lastPID = millis();
long dt = 0;

bool offLine = true;
long distance = 0;

enum direction {
  FORWARD,
  LEFT,
  RIGHT,
  BACKWARD
};

int currentIntersection = 5;
direction intersectionTurns[15] = { LEFT, LEFT, LEFT, FORWARD, LEFT, LEFT /*NOLINEINTERSECION WONT BE READ*/ /*no line - keep going forward,*/, RIGHT, LEFT, FORWARD, LEFT /*Is now in final dead end*/, FORWARD, FORWARD, FORWARD, LEFT, LEFT };

/*TODO: need to detect forward + left XOR right (-> T ->) crossing (would currently turn, and never go forward)
 Option 1:
 Following line -> at least one turn sensor goes high -> note the sensors that went high, but continue forward until they go low again -> note if forward is still high -> we now know what the intersection looks like, choose correct from possible options
 we should only have slightly overshot the intersection so going left, right or forward is still possible.

 Option 2:
 Treat all turns, including simple left and right turns as intersections, and hard code the right choice for each one.
 Result: Detect right or left turn -> count it as an intersection so look for correct turn in the hard coded path.
 Means hard coding every single turn, but eliminates problem of -> T -> crossings*/

void setup() {
  // put your setup code here, to run once:

  pinMode(MOTORLEFT1, OUTPUT);
  pinMode(MOTORLEFT2, OUTPUT);
  pinMode(MOTORRIGHT1, OUTPUT);
  pinMode(MOTORRIGHT2, OUTPUT);
  digitalWrite(MOTORLEFT1, LOW);
  digitalWrite(MOTORLEFT2, LOW);
  digitalWrite(MOTORRIGHT1, LOW);
  digitalWrite(MOTORRIGHT2, LOW);

  Serial.begin(9600);

  gripperServo.attach(GRIPPERPIN);
  verticalServo.attach(VERTICALPIN);
  gripperServo.write(GRIPPER_DROPOFF_POS);
  verticalServo.write(VERTICAL_DRIVE_POS);
  delay(200);
  gripperServo.detach();
  verticalServo.detach();

  pinMode(LT_L3_PIN, INPUT);
  pinMode(LT_L2_PIN, INPUT);
  pinMode(LT_L1_PIN, INPUT);
  pinMode(LT_R1_PIN, INPUT);
  pinMode(LT_R2_PIN, INPUT);
  pinMode(LT_R3_PIN, INPUT);

  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);

  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), ENCODER_RIGHT_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), ENCODER_RIGHT_B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), ENCODER_LEFT_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), ENCODER_LEFT_B_ISR, CHANGE);

  pinMode(STARTBUTTON, INPUT_PULLUP);

  while (digitalRead(STARTBUTTON) == HIGH)
    ;

  if (DO_CALIBRATION) {
    Serial.println("Starting Calibration!");
    calibrateSensors();
    while (1)
      ;
  } else {
    Serial.println("Reading from EEPROM!");
    CalibrationValues calib;
    EEPROM.get(0, calib);
    memcpy(sensorOnLine, calib.sensorOnLine, sizeof(sensorOnLine));
    memcpy(sensorAlmostOnLine, calib.sensorAlmostOnLine, sizeof(sensorAlmostOnLine));
    memcpy(sensorOffLine, calib.sensorOffLine, sizeof(sensorOffLine));
    for (int i = 0; i < 6; i++) {
      Serial.print(" OnLine values pin " + String(i) + ": " + String(sensorOnLine[i]));
      Serial.print(" AlmostOnLine values pin " + String(i) + ": " + String(sensorAlmostOnLine[i]));
      Serial.println(" OffLine values pin " + String(i) + ": " + String(sensorOffLine[i]));
    }
  }

  startTime, dt = millis();
}

void loop() {
  //TODO: Might be good to normalize sensor readings, would require a calibration step where the robot sweeps over the line to see highest and lowest reading for each sensor.
  //Serial.println(encoderLeft);
  //Serial.println(encoderRight);
  long sum = 0;
  offLine = true;

  /*if(millis() - startTime > 2000){
    Serial.print(millis() - startTime);
    Serial.println("Amount of PIDs: " + String(amountOfPIDS));
    //startTime = millis();
  }*/
  for (int i = 1; i < 5; i++) {
    int raw = analogRead(sensorPins[i]);
    if (raw > sensorOffLine[i]) {
      offLine = false;
    }
  }

  int leftTurnSensor = analogRead(sensorPins[0]);
  int rightTurnSensor = analogRead(sensorPins[5]);
  // Read distance

  if (DEBUG_TURN_SENSORS) {
    Serial.println("Left turn sensor: " + String(leftTurnSensor));
    Serial.println("Right turn sensor: " + String(rightTurnSensor));
  }

  if (leftTurnSensor > sensorOnLine[0]) {
    if (DEBUG_FLAGS && hasLeftTurn == 0) {
      Serial.println("I detected a left turn, flagging");
    }
    hasLeftTurn = 1;
  }
  if (rightTurnSensor > sensorOnLine[5]) {
    if (DEBUG_FLAGS && hasRightTurn == 0) {
      Serial.println("I detected a right turn, flagging");
    }
    hasRightTurn = 1;
  }

  // check for forward intersection (might be forward + left XOR right)
  if (hasLeftTurn || hasRightTurn) {  // searching for intersection
    // Option 1:
    // go set distance (thickness of line) to look for if forward exists or not

    // Option 2:
    // Keep going until turn sensors go back to low. This means we overshot the intersection. Look for forward, we now have entire intersection information. Make informed turn descision.
    followLine();

    // read sensors again to se when we exit the intersection
    leftTurnSensor = analogRead(sensorPins[0]);
    rightTurnSensor = analogRead(sensorPins[5]);
    if (leftTurnSensor <= ON_LINE && rightTurnSensor <= ON_LINE) {
      delay(150);
      // We overshot the intersection, check for forward option
      if (analogRead(sensorPins[2]) > sensorAlmostOnLine[2] || analogRead(sensorPins[3]) > sensorAlmostOnLine[3]) {
        hasForwardTurn = 1;
      } else {
        hasForwardTurn = 0;
      }

      direction chosenTurn;
      // We now have entire intersection (or lack there of), make choice
      // if we have 2 or more options, we are in an intersection
      if ((hasLeftTurn + hasRightTurn + hasForwardTurn) >= 2) {
        // We found intersection, use map to choose turn
        chosenTurn = intersectionTurns[currentIntersection];
        currentIntersection++;
        Serial.println("Intersection detected: " + String(currentIntersection));
      } else {
        // Only one possible turn, so choose it
        if (hasRightTurn) {
          chosenTurn = RIGHT;
        } else if (hasLeftTurn) {
          chosenTurn = LEFT;
        } else if (hasForwardTurn) {
          chosenTurn = FORWARD;
        }
      }

      switch (chosenTurn) {
        case LEFT:
          if (DEBUG_TURNING) {
            Serial.println("Switch case turning left");
          }
          turnLeft();
          break;
        case RIGHT:
          if (DEBUG_TURNING) {
            Serial.println("Switch case turning right");
          }
          turnRight();
          break;
        case FORWARD:
          if (DEBUG_TURNING) {
            Serial.println("Switch case FORWARD");
          }
          followLine();
          break;
      }
      // We are done with intersection, reset intersection checkers
      hasLeftTurn = 0;
      hasRightTurn = 0;
      hasForwardTurn = 0;
    }

    return;  // Dont do normal PID until we have solved the intersection.
  }
  if (offLine) {
    //Serial.println("sum: " + String(sum));
    noLineLogic();
  }

  // Follow line PID
  followLine();
}

void followLine() {
  if (millis() - lastTime > 20) {
    distance = readDistance();
    lastTime = millis();
    if (DEBUG_DISTANCE_SENSORS) {
      Serial.println("Distance: " + String(distance));
    }
  }

  if (distance <= DISTANCE_FROM_GRIPPER) {  // Might be inacurate when close range in practice so might have to do it "blind", i.e move forward x amount, then do closing
    Serial.println("Object too close, grabbing it!");
    pickUpAndStore();
  }

  int error = calulateWeightedError();
  int correction = calculatePID(error);

  //Test for max speed
  int leftMotor = constrain(baseSpeed + correction, 0, 255);
  int rightMotor = constrain(baseSpeed - correction, 0, 255);

  driveMotors(leftMotor, rightMotor);
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
                                 // The ping travels out and back, so to find the distance of the object we
                                 // take half of the distance travelled.
}

void pickUpAndStore() {
  analogWrite(MOTORLEFT1, 0);   // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 0);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  gripperServo.attach(GRIPPERPIN);
  verticalServo.attach(VERTICALPIN);
  //Serial.println("Vertical down position!");
  verticalServo.write(VERTICAL_PRE_OPEN_POS);
  delay(400);
  gripperServo.write(GRIPPER_OPEN_POS);
  delay(400);
  verticalServo.write(VERTICAL_DOWN_POS);
  delay(500);
  gripperServo.write(GRIPPER_CLOSED_POS);
  //Serial.println("CLOSING GRIPPERS!");
  delay(500);
  verticalServo.write(VERTICAL_UP_POS);
  //Serial.println("MOVING GRIPPERS UP!");
  delay(1200);
  gripperServo.write(GRIPPER_DROPOFF_POS);
  delay(200);
  //Serial.println("OPENING GRIPPERS UP!");
  verticalServo.write(VERTICAL_DRIVE_POS);
  delay(200);
  gripperServo.detach();
  verticalServo.detach();
}

int readDistance() {
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);

  return (microsecondsToCentimeters(pulseIn(ECHOPIN, HIGH)));
}

void driveMotors(int left, int right) {
  analogWrite(MOTORLEFT1, left);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);     // Speed (0-255)

  // RIGHT motor
  analogWrite(MOTORRIGHT1, right);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  if (DEBUG_MOTOR_SPEED) {
    Serial.print("Left motor: ");
    Serial.println(left);
    Serial.print("Right motor: ");
    Serial.println(right);
  }
}

int calulateWeightedError() {
  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < 6; i++) {
    int raw = analogRead(sensorPins[i]);

    // raw = constrain(raw, sensorMin[i], sensorMax[i]);
    // int normalized = map(raw, sensorMin[i], sensorMax[i], 0, 1000);

    int weight = -weightArray[i];  // calculate weight for the pin (-3500 -2500 -1500 -500 +500 +1500 +2500 +3500)
    weightedSum += (long)raw * weight;
    // weightedSum += normalized * weight; // Using normalized readings
    sum += raw;  // Sum all readings (so we can normalize, as we dont care about how dark/light just relation to each other)
  }

  // Will likely never happen (means completley lost line, and all white surface has 0 reflection)
  // If it happens turn fully left or right
  if (sum == 0) {
    return lastError > 0 ? 3500 : -3500;
  }

  return weightedSum / sum;
}

int calculatePID(int error) {
  amountOfPIDS++;
  float e = (float)error;
  //integral += e;
  dt = millis() - lastPID;
  float derivative = (e - lastError) / dt;
  lastPID = millis();
  lastError = e;
  float correction = Kp * e /*+ Ki * integral*/ + Kd * derivative;
  //Maybe could remove if we constrain it before the motors
  //correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  if (DEBUG_PID) {
    Serial.println("dt: " + String(dt));
    Serial.println("Error: " + String(e));
    Serial.print("P-term: ");
    Serial.println(e * Kp);
    /*Serial.print("I-term: ");
    Serial.println(integral * Ki);*/
    Serial.print("D-term: ");
    Serial.println(Kd * derivative);
    Serial.println("Correction: " + String(correction));
  }
  return (int)correction;
}

void turnRight() {
  // encoderRight = 0;
  // encoderLeft = 0;
  if (DEBUG_LINE_SENSORS) {
    for (int i = 0; i < 6; i++) {
      int raw = analogRead(sensorPins[i]);
      if (DEBUG_LINE_SENSORS) {
        Serial.print("Raw pin ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(raw);
      }
    }
  }
  Serial.println("I am turning right!");
  analogWrite(MOTORRIGHT1, 0);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 150);
  analogWrite(MOTORLEFT1, 200);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);    // Speed (0-255)

  delay(200);
  while (analogRead(sensorPins[4]) < sensorOnLine[4])
    ;
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 255);
  analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 255);  // Speed (0-255)
  delay(50);

  forcePID(200);
}

void turnLeft() {
  //encoderRight = 0;
  //encoderLeft = 0;
  if (DEBUG_LINE_SENSORS) {
    for (int i = 0; i < 6; i++) {
      int raw = analogRead(sensorPins[i]);
      if (DEBUG_LINE_SENSORS) {
        Serial.print("Raw pin ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(raw);
      }
    }
  }
  Serial.println("I am turning left!");
  analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 150);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 200);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  delay(200);
  while (analogRead(sensorPins[1]) < sensorOnLine[1])
    ;
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 255);
  analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 255);  // Speed (0-255)
  delay(50);
  forcePID(200);
}

void forcePID(int amountOfCycles) {
  for (int i = 0; i < amountOfCycles; i++) {
    followLine();
  }
}

void noLineLogic() {
  Serial.println("Start of noLineLogic()");
  float timeSinceStart = millis();
  bool foundLine = false;
  driveMotors(170, 160);
  while (readDistance() > 13 && !foundLine) {
    if (millis() - timeSinceStart > 1500) {
      if (lineFinder()) {
        Serial.println("Found the line again!");
        foundLine = true;
        forcePID(200);
        break;
      } else {
        driveMotors(170, 160);
        delay(100);
      }
    }
    for (int i = 0; i < 6; i++) {
      if (analogRead(sensorPins[i]) > sensorOnLine[i]) {
        foundLine = true;
        break;
      }
    }
    delay(10);
  }
  if (foundLine) {
    Serial.println("Time since start of noLineLogic(): " + String(millis() - timeSinceStart));
    return;
  } else if (currentIntersection == 5) {
    Serial.println("I am turning left at intersection 5!");

    analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
    analogWrite(MOTORLEFT2, 200);   // Speed (0-255)
    analogWrite(MOTORRIGHT1, 200);  // HIGH = forward, change if reversed
    analogWrite(MOTORRIGHT2, 0);

    delay(400);
    forcePID(400);
    currentIntersection++;

  } else if (currentIntersection >= 14) {
    finalDance();
  } else {
    uTurn();
  }
}

void uTurn() {
  if (DEBUG_LINE_SENSORS) {
    for (int i = 0; i < 6; i++) {
      int raw = analogRead(sensorPins[i]);
      if (DEBUG_LINE_SENSORS) {
        Serial.print("Raw pin ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(raw);
      }
    }
  }
  Serial.println("I am doing a u-turn!!");
  analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 200);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 200);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  delay(200);
  while (analogRead(sensorPins[0]) < sensorOnLine[0])
    ;

  analogWrite(MOTORLEFT1, 255);   // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 255);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 255);
  delay(100);
  forcePID(200);
}

bool lineFinder() {
  Serial.println("In lineFinder()");
  //TURNING LEFT FIRST
  analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 200);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 200);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);
  float timeDelay = millis();
  while (millis() - timeDelay < 300) {
    for (int i = 0; i < 6; i++) {
      if (analogRead(sensorPins[i]) > sensorOnLine[i]) {
        return true;
      }
    }
  }
  //TURNING RIGHT AFTER
  analogWrite(MOTORLEFT1, 200);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);    // Speed (0-255)
  analogWrite(MOTORRIGHT1, 0);   // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 200);
  timeDelay = millis();
  while (millis() - timeDelay < 600) {
    for (int i = 0; i < 6; i++) {
      if (analogRead(sensorPins[i]) > sensorOnLine[i]) {
        return true;
      }
    }
  }
  return false;
}

void finalDance() {
  Serial.println("I am turning left!");
  analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);    // Speed (0-255)
  analogWrite(MOTORRIGHT1, 0);   // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 255);
  delay(2000);
  driveMotors(0, 0);

  gripperServo.attach(GRIPPERPIN);
  verticalServo.attach(VERTICALPIN);
  verticalServo.write(VERTICAL_PRE_OPEN_POS);
  //Serial.println("Vertical down position!");
  gripperServo.write(GRIPPER_OPEN_POS);
  delay(200);
  gripperServo.write(GRIPPER_CLOSED_POS);
  delay(300);
  gripperServo.write(GRIPPER_OPEN_POS);
  delay(200);
  gripperServo.write(GRIPPER_CLOSED_POS);
  delay(300);
  gripperServo.write(GRIPPER_OPEN_POS);
  delay(200);
  gripperServo.write(GRIPPER_CLOSED_POS);
  delay(300);
  gripperServo.write(GRIPPER_OPEN_POS);
  delay(200);
  gripperServo.write(GRIPPER_CLOSED_POS);
  delay(300);
  verticalServo.write(VERTICAL_DOWN_POS);
  delay(500);
  verticalServo.write(VERTICAL_UP_POS);
  delay(500);
  gripperServo.detach();
  verticalServo.detach();
  analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);    // Speed (0-255)
  analogWrite(MOTORRIGHT1, 0);   // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 255);
  delay(2000);
  driveMotors(0, 0);
  while (1)
    ;
}

void calibrateSensors() {
  int sensorReading = 0;
  int maxSensorValues[6] = { 0, 0, 0, 0, 0, 0 };
  int minSensorValues[6] = { 2000, 2000, 2000, 2000, 2000, 2000 };
  analogWrite(MOTORLEFT1, 80);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 0);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 80);
  for (int i = 0; i < 700; i++) {
    for (int j = 0; j < 6; j++) {
      sensorReading = analogRead(sensorPins[j]);
      if (sensorReading > maxSensorValues[j]) {
        maxSensorValues[j] = sensorReading;
      }
      if (sensorReading < minSensorValues[j]) {
        minSensorValues[j] = sensorReading;
      }
    }
  }
  analogWrite(MOTORLEFT1, 0);    // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 80);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 80);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  for (int i = 0; i < 1400; i++) {
    for (int j = 0; j < 6; j++) {
      sensorReading = analogRead(sensorPins[j]);
      if (sensorReading > maxSensorValues[j]) {
        maxSensorValues[j] = sensorReading;
      }
      if (sensorReading < minSensorValues[j]) {
        minSensorValues[j] = sensorReading;
      }
    }
  }

  analogWrite(MOTORLEFT1, 80);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 0);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 80);

  for (int i = 0; i < 1400; i++) {
    for (int j = 0; j < 6; j++) {
      sensorReading = analogRead(sensorPins[j]);
      if (sensorReading > maxSensorValues[j]) {
        maxSensorValues[j] = sensorReading;
      }
      if (sensorReading < minSensorValues[j]) {
        minSensorValues[j] = sensorReading;
      }
    }
  }

  analogWrite(MOTORLEFT1, 0);    // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 80);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 80);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  for (int i = 0; i < 700; i++) {
    for (int j = 0; j < 6; j++) {
      sensorReading = analogRead(sensorPins[j]);
      if (sensorReading > maxSensorValues[j]) {
        maxSensorValues[j] = sensorReading;
      }
      if (sensorReading < minSensorValues[j]) {
        minSensorValues[j] = sensorReading;
      }
    }
  }
  driveMotors(0, 0);

  for (int i = 0; i < 6; i++) {
    int sensorDiff = maxSensorValues[i];
    sensorOnLine[i] = sensorDiff * ON_LINE_FACTOR;
    sensorAlmostOnLine[i] = sensorDiff * ALMOST_ON_LINE_FACTOR;
    sensorOffLine[i] = sensorDiff * OFF_LINE_FACTOR;

    Serial.print(" OnLine values pin " + String(i) + ": " + String(sensorOnLine[i]));
    Serial.print(" AlmostOnLine values pin " + String(i) + ": " + String(sensorAlmostOnLine[i]));
    Serial.println(" OffLine values pin " + String(i) + ": " + String(sensorOffLine[i]));
  }

  CalibrationValues calib;

  memcpy(calib.sensorOnLine, sensorOnLine, sizeof(sensorOnLine));
  memcpy(calib.sensorAlmostOnLine, sensorAlmostOnLine, sizeof(sensorAlmostOnLine));
  memcpy(calib.sensorOffLine, sensorOffLine, sizeof(sensorOffLine));

  EEPROM.put(0, calib);
}

void ENCODER_RIGHT_A_ISR() {
  int stateA = digitalRead(ENCODER_RIGHT_A);
  int stateB = digitalRead(ENCODER_RIGHT_B);

  // If A leads B, forward; else reverse
  if (stateA == stateB) encoderRight++;
  else encoderRight--;
}

void ENCODER_RIGHT_B_ISR() {
  int stateA = digitalRead(ENCODER_RIGHT_A);
  int stateB = digitalRead(ENCODER_RIGHT_B);

  if (stateA != stateB) encoderRight++;
  else encoderRight--;
}

void ENCODER_LEFT_A_ISR() {
  int stateA = digitalRead(ENCODER_LEFT_A);
  int stateB = digitalRead(ENCODER_LEFT_B);

  // If A leads B, forward; else reverse
  if (stateA == stateB) encoderLeft++;
  else encoderLeft--;
}

void ENCODER_LEFT_B_ISR() {
  int stateA = digitalRead(ENCODER_LEFT_A);
  int stateB = digitalRead(ENCODER_LEFT_B);

  if (stateA != stateB) encoderLeft++;
  else encoderLeft--;
}
