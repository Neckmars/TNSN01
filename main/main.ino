#include <Servo.h>
#include <EEPROM.h>
#include <Vector.h>

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

// Orientation definitions
#define NORTH 0
#define WEST 1
#define SOUTH 2
#define EAST 3

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

float Kp = 0.20;
float Ki = 0.00024;
float Kd = 10;

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

// A star node class and variables
int currentNewIntersection = -1;
int cylindersCollected = 1;
int currentOrientation = WEST;

class Node {
public:
    int x = -1;
    int y = -1;
    int h = 0;
    int g = 0;

    // Order: North, South, West, East
    int northLimit;
    int southLimit;
    int westLimit;
    int eastLimit;


    Node *parent = nullptr;

    Node() {};

    void SetParams(int x, int y, int northLimit, int southLimit, int westLimit, int eastLimit) {
        this->x = x;
        this->y = y;

        this->northLimit = northLimit;
        this->southLimit = southLimit;
        this->westLimit = westLimit;
        this->eastLimit = eastLimit;
    }

    void resetParams(){
        this->h = 0;
        this->g = 0;
        this->parent = nullptr;
    }

    int f() {
        return g + h;
    }
};

// Define Node positions
Node MapNodes[7][7];
Node* intersectionNodes[14]= {&MapNodes[1][2], &MapNodes[1][1],
                                &MapNodes[1][1], &MapNodes[6][3],
                                &MapNodes[6][3], &MapNodes[4][4],
                                &MapNodes[3][4], &MapNodes[1][1],
                                &MapNodes[1][2], &MapNodes[2][3],
                                &MapNodes[2][4], &MapNodes[2][4],
                                &MapNodes[2][3], &MapNodes[1][2]};
Node* finalCylinderNode = &MapNodes[2][6];

// Define help structures 
struct importantVectors {
        Vector<Node*> v1;
        Vector<direction> v2;
}; 
struct dobleInt {
    int value1;
    direction value2;
};
/*struct ordersAndIndex {
    Vector<int> order;
    int index;
};*/


// Intersection turns
int currentIntersection = 0;
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

  //Commenting out encoders, we don't use them anyways
  /*
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  */
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  //Commenting out encoders, we don't use them anyways
  /*
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), ENCODER_RIGHT_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), ENCODER_RIGHT_B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), ENCODER_LEFT_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), ENCODER_LEFT_B_ISR, CHANGE);
  */
  pinMode(STARTBUTTON, INPUT_PULLUP);

  // Create virtual map for the robot
  createMap();

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

  // Only enters when the number of cyllinders collected are 2 and currentNewIntersection hasn't been used yet
  if (cylindersCollected == 2 && currentNewIntersection == -1) {
    Serial.println("Calculating AStar");
    auto [newMaxNodes, specialNodePos] = Astar_robot(currentIntersection, 0, currentOrientation);
    currentNewIntersection = 0;
    currentIntersection = 0;
  }

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
    if (leftTurnSensor <= sensorOffLine[0] && rightTurnSensor <= sensorOffLine[5]) {
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
        Serial.println("Intersection detected: " + String(currentIntersection) + " Index: " + String(currentIntersection - 1));
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
        // Only a Astar case
        case BACKWARD:
          if (DEBUG_TURNING) {
            Serial.println("Switch case BACKWARD");
          }
          uTurn();
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
    cylindersCollected++;
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
  currentOrientation = (currentOrientation-1)%4;
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
  currentOrientation = (currentOrientation+1)%4;
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

void forcePID(int amountOfMillis) {
  long startTime = millis();
  while (millis() -  startTime < amountOfMillis) {
    followLine();
  }
}

void noLineLogic() {
  Serial.println("Start of noLineLogic()");
  float timeSinceStart = millis();
  bool foundLine = false;
  driveMotors(170, 160);
  while (readDistance() > 13 && !foundLine) {
    if (millis() - timeSinceStart > 1400) {
      if (lineFinder()) {
        Serial.println("Found the line again!");
        foundLine = true;
        forcePID(350);
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
    if (millis() - timeSinceStart <= 300) {
      uTurn();
    }
  }
}

void uTurn() {
  currentOrientation = (currentOrientation+2)%4;
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
  while (millis() - timeDelay < 600) {
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
  while (millis() - timeDelay < 1200) {
    for (int i = 0; i < 6; i++) {
      if (analogRead(sensorPins[i]) > sensorOnLine[i]) {
        return true;
      }
    }
  }
  //TURNING LEFT THIRD TIME
  analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 200);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 200);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);
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

    Serial.print(" Raw values pin " + String(i) + ": " + String(maxSensorValues[i]));
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

// A star functions
void resetMap() {
    for(int j = 0; j < 7; j++){
        for (int i = 0; i < 7; i++) {
            MapNodes[j][i].resetParams();
        }
    }
}

void createMap() {
    // Order: North, South, West, East
    // Generate first line
    MapNodes[0][0].SetParams(0, 0, 0, 0, 0, 1);
    MapNodes[0][1].SetParams(1, 0, 1, 0, 1, 0);
    MapNodes[0][2].SetParams(2, 0, 1, 0, 0, 1);
    MapNodes[0][3].SetParams(3, 0, 0, 0, 1, 0);
    MapNodes[0][4].SetParams(4, 0, 1, 0, 0, 1);
    MapNodes[0][5].SetParams(5, 0, 0, 0, 1, 1);
    MapNodes[0][6].SetParams(6, 0, 1, 0, 1, 0);

    // Generate second line
    MapNodes[1][0].SetParams(0, 1, 1, 0, 0, 1);
    MapNodes[1][1].SetParams(1, 1, 1, 1, 1, 1);
    MapNodes[1][2].SetParams(2, 1, 0, 1, 1, 1);
    MapNodes[1][3].SetParams(3, 1, 1, 0, 1, 0);
    MapNodes[1][4].SetParams(4, 1, 0, 1, 0, 1);
    MapNodes[1][5].SetParams(5, 1, 1, 0, 1, 0);
    MapNodes[1][6].SetParams(6, 1, 1, 1, 0, 0);

    // Generate third line
    MapNodes[2][0].SetParams(0, 2, 1, 1, 0, 0);
    MapNodes[2][1].SetParams(1, 2, 1, 1, 0, 0);
    MapNodes[2][2].SetParams(2, 2, 0, 0, 0, 1);
    MapNodes[2][3].SetParams(3, 2, 0, 1, 1, 1);
    MapNodes[2][4].SetParams(4, 2, 1, 0, 1, 1);
    MapNodes[2][5].SetParams(5, 2, 0, 1, 1, 0);
    MapNodes[2][6].SetParams(6, 2, 1, 1, 0, 0);
    
    // Generate fourth line
    MapNodes[3][0].SetParams(0, 3, 1, 1, 0, 0);
    MapNodes[3][1].SetParams(1, 3, 1, 1, 0, 0);
    MapNodes[3][2].SetParams(2, 3, 1, 0, 0, 1);
    MapNodes[3][3].SetParams(3, 3, 0, 0, 1, 1);
    MapNodes[3][4].SetParams(4, 3, 1, 1, 1, 0);
    MapNodes[3][5].SetParams(5, 3, 1, 0, 0, 1);
    MapNodes[3][6].SetParams(6, 3, 0, 1, 1, 0);

    // Generate fifth line
    MapNodes[4][0].SetParams(0, 4, 1, 1, 0, 0);
    MapNodes[4][1].SetParams(1, 4, 0, 1, 0, 1);
    MapNodes[4][2].SetParams(2, 4, 0, 1, 1, 0);
    MapNodes[4][3].SetParams(3, 4, 1, 0, 0, 1);
    MapNodes[4][4].SetParams(4, 4, 1, 1, 1, 0);
    MapNodes[4][5].SetParams(5, 4, 0, 1, 0, 1);
    MapNodes[4][6].SetParams(6, 4, 0, 0, 1, 0);

    // Generate sixth line
    MapNodes[5][0].SetParams(0, 5, 1, 1, 0, 0);
    MapNodes[5][1].SetParams(1, 5, 1, 0, 0, 1);
    MapNodes[5][2].SetParams(2, 5, 1, 0, 1, 0);
    MapNodes[5][3].SetParams(3, 5, 1, 1, 0, 0);
    MapNodes[5][4].SetParams(4, 5, 0, 1, 0, 1);
    MapNodes[5][5].SetParams(5, 5, 0, 0, 1, 1);
    MapNodes[5][6].SetParams(6, 5, 1, 0, 1, 0);

    // Generate seventh line
    MapNodes[6][0].SetParams(0, 6, 0, 1, 0, 1);
    MapNodes[6][1].SetParams(1, 6, 0, 1, 1, 0);
    MapNodes[6][2].SetParams(2, 6, 0, 1, 0, 1);
    MapNodes[6][3].SetParams(3, 6, 0, 1, 1, 1);
    MapNodes[6][4].SetParams(4, 6, 0, 0, 1, 0);
    MapNodes[6][5].SetParams(5, 6, 0, 0, 0, 1);
    MapNodes[6][6].SetParams(6, 6, 0, 1, 1, 0);

    return;
}

int ManhattanDistance(int x1, int y1, int x2, int y2) {
    return (abs(x1 - x2) + abs(y1 - y2));
}

Vector<Node*> GetNeighbors(Node* thisNode) {
    Vector<Node*> vectorOut;
    Serial.println("");
    if (thisNode->northLimit == 1) vectorOut.push_back(&MapNodes[thisNode->y+1][thisNode->x]);
    if (thisNode->southLimit == 1) vectorOut.push_back(&MapNodes[thisNode->y-1][thisNode->x]);
    if (thisNode->westLimit == 1) vectorOut.push_back(&MapNodes[thisNode->y][thisNode->x-1]);
    if (thisNode->eastLimit == 1) vectorOut.push_back(&MapNodes[thisNode->y][thisNode->x+1]);

    return vectorOut;
}

Vector<Node*> AStar_Algorithm(Node* StartNode, Node* GoalNode) {
    Vector<Node*> open;
    Vector<Node*> closed;

    resetMap();
    Node* start = StartNode;
    
    start->g = 0;
    start->h = ManhattanDistance(StartNode->x, StartNode->y, GoalNode->x, GoalNode->y);
    start->parent = nullptr;
    open.push_back(start);

    while(!open.empty()) {

        // Verify and choose the lowest f
        Node* current = open[0];
        for (Node* n: open) {
            if (n->f() < current->f()) current = n;
        }

        // In case we are in the goal, find the parents of the nodes until get to the start node
        if(current->x == GoalNode->x && current->y == GoalNode->y) {
            Vector<Node*> path;
            while(current != nullptr) {
                path.push_back(current);
                current = current->parent;
                Serial.print("x: ");
                Serial.print(current->x);
                Serial.print(" y: ");
                Serial.println(current->y);
            }
            // Reverse the path to obtain the right order
            reverseVector(path);
            // Return final path
            return path;
        }

        // Move the current node to closed
        open.remove(current);
        closed.push_back(current);

        // Identify neighbors and explore them
        Vector<Node*> neighbors = GetNeighbors(current);
        for (Node* neighbor : neighbors) {
            int neighbor_g = current->g + 1;
            bool skip = false;
            bool inOpen = false;
            
            // Verify if the node is in the open vector
            for (Node* o : open) {
                // If the current cost is greater or equal than the cost that Node already has,
                // move to the next neighbor 
                if (o == neighbor) {
                    inOpen = true;
                    // Update neighbor only if it finds a better path
                    if (neighbor_g < o->g) {
                        o->g = neighbor_g;
                        o->parent = current;
                    }
                    break;
                }
            }

            // Verify if the node is in the closed vector
            for (Node* c : closed) {
                // If the current cost is greater or equal than the cost that Node already has,
                // move to the next neighbor 
                if (c == neighbor) {
                    skip = true;
                    break;
                }
            }
            if (skip) continue;

            // In case the neighbor is neither in the open nor the closed vector,
            // add it to the open vector
            if (!inOpen) {
                neighbor->g = neighbor_g;
                neighbor->h = ManhattanDistance(neighbor->x, neighbor->y, GoalNode->x, GoalNode->y);
                neighbor->parent = current;
                open.push_back(neighbor);
            }

        }
    }
  return {};
}

    // Functions to translate the Node Vector into positions and orders
importantVectors translateNodes2Orders(Vector<Node*> Path, int originalOrientation) {
    Vector<Node*> final_nodes;
    Vector<direction> final_orders;
    Vector<int> tempOrders;
    int n;
    int currentOrientation = originalOrientation;
    int nextOrientation;
    for (int i = 0; i < Path.size()-1; i++) {
        Node* p = Path[i];
        auto [n, nextOrientation] = getNextOrder(Path[i], Path[i+1], currentOrientation);
        tempOrders.push_back(n);
        //cout << nextOrientation << " \n";
        currentOrientation = nextOrientation;
    }

    for (int i = 0; i < Path.size()-1; i++) {
        Node* p = Path[i];
        int ord = tempOrders[i];
        if (p->northLimit+p->southLimit+p->eastLimit+p->westLimit > 2) {
            final_nodes.push_back(p);
            if(ord == 0) final_orders.push_back(FORWARD);
            if(ord == -1) final_orders.push_back(LEFT);
            if(ord == 1) final_orders.push_back(RIGHT);
            if(ord == 2) final_orders.push_back(BACKWARD);
        }
    }

    return importantVectors { final_nodes, final_orders };
}

dobleInt getNextOrder(Node* currentNode, Node* nextNode, int originalOrientation) {
    int change_y = nextNode->y - currentNode->y;
    int change_x = nextNode->x - currentNode->x;

    int currentOrientation = originalOrientation;
    int targetOrientation;
    int order;

    if (change_y == 1)          targetOrientation = NORTH;
    else if (change_y == -1)    targetOrientation = SOUTH;
    else if (change_x == 1)     targetOrientation = EAST;
    else if (change_x == -1)    targetOrientation = WEST;

    int diff = (targetOrientation - currentOrientation + 4) % 4;

    switch (diff) {
    case 0:
        order = FORWARD;
        break;
    case 1:
        order = RIGHT;
        break;

    case 3:
        order = LEFT;
        break;
    
    case 2:
        order = BACKWARD;
        break;

    default:
        break;
    }
    currentOrientation = targetOrientation;
    return dobleInt {order, currentOrientation};
}

dobleInt Astar_robot(int currentIntersection, int distanceFromIntersection, int orientation){
    // define 2 paths
    // path 1: from the last node to the goal node (distance = path + distanceFromIntersaction)
    // path 2: from the following node, to the (distance = path + (distanceBetweenIntersactions - distanceFromIntersaction))
    // choose the path with the lowest distance
    
    // specialNode will tell us if the robot will cross the node without dark line. If it doesn't cross that node, or if it does it and the order is to go forward, the number will stay at -1
    // Otherwise the number will corespond to the number of the order.

    int specialNode = -1;
    Serial.println("Path 1 calculation");
    Vector<Node*> path1 = AStar_Algorithm(intersectionNodes[currentIntersection], finalCylinderNode);
    Serial.println("Path 2 calculation");
    Vector<Node*> path2 = AStar_Algorithm(intersectionNodes[currentIntersection + 1], finalCylinderNode);
    Serial.println(path1[0]->x);
    Serial.println(path2[0]->x);
    importantVectors myVectors;

    if (path1.size() > path2.size()) {
        myVectors = translateNodes2Orders(path2, orientation);
    }
    else {
        orientation += 2;
        myVectors = translateNodes2Orders(path1, orientation);       
    }

    auto& [finalNodes, finalOrders] = myVectors;

    for(int i = 0; i < finalNodes.size(); i++) {
        Node* c_node = finalNodes[i];
        int c_order = finalOrders[i];
        if (c_node->x == 4 && c_node->y == 4) {
            if (c_order == 0) finalOrders.remove(i);
            else specialNode = i;
        }  
    }

    if (path1.size() <= path2.size()) {
      insertAtFront(finalOrders, BACKWARD);
    }

    finalOrders.push_back(FORWARD);
    finalOrders.push_back(LEFT);
    finalOrders.push_back(LEFT);

    int n = finalOrders.size();
    Serial.println(n);

    // Testing if this will change the whole intersection array

    for (int i = 0; i < n; i++) {
      intersectionTurns[i] = finalOrders[i];
      if(intersectionTurns[i] == FORWARD) Serial.print("forward");
      if(intersectionTurns[i] == BACKWARD) Serial.print("backward");
      if(intersectionTurns[i] == LEFT) Serial.print("left");
      if(intersectionTurns[i] == RIGHT) Serial.print("right");
    }

    return dobleInt {n, specialNode};
}

// Function to make vectors work better, should've used another collective class
template<typename T>
void reverseVector(Vector<T>& v) {
    int n = v.size();
    for (int i = 0; i < n / 2; i++) {
        T temp = v[i];
        v[i] = v[n - 1 - i];
        v[n - 1 - i] = temp;
    }
}

template<typename T>
void insertAtFront(Vector<T>& v, const T& value) {
    int n = v.size();
    v.push_back(value);  // make space
    
    for (int i = n; i > 0; i--) {
        v[i] = v[i - 1];
    }
    
    v[0] = value;
}

// pls don't explode
  //Commenting out encoders, we don't use them anyways
/*
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
}*/
