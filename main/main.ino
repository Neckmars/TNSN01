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

#define ON_LINE_FACTOR .9
#define ALMOST_ON_LINE_FACTOR .85
#define OFF_LINE_FACTOR .5

#define TRIGPIN 8
#define ECHOPIN 7

#define GRIPPERPIN 9
#define VERTICALPIN 10

#define GRIPPER_CLOSED_POS 40
#define GRIPPER_OPEN_POS 140
#define GRIPPER_DROPOFF_POS 75
#define VERTICAL_DOWN_POS 175
#define VERTICAL_PRE_OPEN_POS 105
#define VERTICAL_UP_POS 60
#define VERTICAL_DRIVE_POS 85
#define DISTANCE_FROM_GRIPPER 11.5

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

#define MAX_NODES 49
#define MAX_ORDERS 30

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
int8_t hasLeftTurn = 0;
int8_t hasRightTurn = 0;
int8_t hasForwardTurn = 0;

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

float Kp = 0.3;
float Ki = 0.00024;
float Kd = 10;

const int8_t MAX_CORRECTION = 45;

int baseSpeed = 210;

/*volatile long encoderRight = 0;
volatile long encoderLeft = 0;
*/

long lastTime = millis();
long lastPID = millis();
long dt = 0;

bool offLine = true;
long distance = 0;

enum direction {
  FORWARD = 0,
  LEFT = 1,
  RIGHT = 2,
  BACKWARD = 3
};

// A star node class and variables
int8_t AStarActivated = 0;
int8_t currentOrientation = WEST;
int8_t newPathSize = -1;
int8_t specialIntersectionIndex = 5;

class Node {
public:
  int8_t x = -1;
  int8_t y = -1;
  int h = 0;
  int g = 0;

  // Order: North, South, West, East
  int8_t northLimit;
  int8_t southLimit;
  int8_t westLimit;
  int8_t eastLimit;


  Node* parent = nullptr;

  void SetParams(int8_t x, int8_t y, int8_t northLimit, int8_t southLimit, int8_t westLimit, int8_t eastLimit) {
    this->x = x;
    this->y = y;

    this->northLimit = northLimit;
    this->southLimit = southLimit;
    this->westLimit = westLimit;
    this->eastLimit = eastLimit;
  }

  void resetParams() {
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
Node* intersectionNodes[16] = { &MapNodes[0][3], &MapNodes[1][2], &MapNodes[1][1],
                                &MapNodes[1][1], &MapNodes[6][3],
                                &MapNodes[6][3], &MapNodes[4][4],
                                &MapNodes[3][4], &MapNodes[1][1],
                                &MapNodes[1][2], &MapNodes[2][3],
                                &MapNodes[2][3], &MapNodes[2][4],
                                &MapNodes[2][4], &MapNodes[2][3], &MapNodes[1][2] };
Node* finalCylinderNode = &MapNodes[2][6];

// Define help structures
struct importantVectors {
  Node* nodes[MAX_NODES];
  int8_t orders[MAX_ORDERS];
  int8_t size;
};
struct ordersAndIndex {
  direction orders[MAX_ORDERS];
  int8_t size;
  int8_t index;
};
struct OrderResult {
  direction order;
  int8_t newOrientation;
};

int8_t countCylinders = 0;

// Intersection turns
int8_t currentIntersection = 0;

long delayForNoLine = 0;


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

  dt = millis();
}

void loop() {
  //TODO: Might be good to normalize sensor readings, would require a calibration step where the robot sweeps over the line to see highest and lowest reading for each sensor.
  //Serial.println(encoderLeft);
  //Serial.println(encoderRight);
  long sum = 0;
  offLine = true;

  // Only enters when the number of cyllinders collected are 2 and currentNewIntersection hasn't been used yet
  /*if (countCylinders == 2 && currentNewIntersection == -1) {
    Serial.println("Calculating AStar");
    ordersAndIndex result = Astar_robot(currentIntersection, 0, currentOrientation);

    newMaxNodes = result.size;
    specialNodePos = result.index;

    currentNewIntersection = 0;
    currentIntersection = 0;
  }*/

  // Look for a sensor reading the line
  for (int i = 0; i < 6; i++) {
    int raw = analogRead(sensorPins[i]);
    if (raw > sensorOffLine[i]) {
      offLine = false;
    }
  }

  int leftTurnSensor = analogRead(sensorPins[0]);
  int rightTurnSensor = analogRead(sensorPins[5]);

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
      delay(70);
      // We overshot the intersection, check for forward option
      hasForwardTurn = 0;

      for (int i = 0; i < 5; i++) {
        if (analogRead(sensorPins[2]) > sensorAlmostOnLine[2] || analogRead(sensorPins[3]) > sensorAlmostOnLine[3]) {
          hasForwardTurn = 1;
          break;
        }
      }

      direction chosenTurn;
      // We now have entire intersection (or lack there of), make choice
      // if we have 2 or more options, we are in an intersection
      if ((hasLeftTurn + hasRightTurn + hasForwardTurn) >= 2) {
        // We found intersection, use map to choose turn
        // Only enters when the number of cyllinders collected are 2 and Astar  hasn't been calculated yet
        if (countCylinders == 2 && !AStarActivated) {
          Serial.print("Calculating AStar");

          // Astar_robot sets path (intersectionTurns) to new path
          ordersAndIndex result = Astar_robot(currentIntersection + 1, 0, currentOrientation);

          newPathSize = result.size;
          Serial.print("New path size: " + String(newPathSize));
          specialIntersectionIndex = result.index;

          AStarActivated = 1;
          currentIntersection = 0;
        }
        chosenTurn = intersectionTurns[currentIntersection];
        if (currentIntersection == 11 && countCylinders != 3) {
          currentIntersection = 13;
        }
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
          forcePID(200);
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
    if (millis() - delayForNoLine > 500) {
      noLineLogic();
      delayForNoLine = millis();
    }
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
  countCylinders++;
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
  delay(500);
  verticalServo.write(VERTICAL_UP_POS);
  delay(1200);
  gripperServo.write(GRIPPER_DROPOFF_POS);
  delay(200);
  verticalServo.write(VERTICAL_DRIVE_POS);
  delay(200);
  gripperServo.detach();
  verticalServo.detach();
  if (countCylinders == 3) {
    uTurn();
  }
}

long readDistance() {
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
  currentOrientation = (currentOrientation - 1 + 4) % 4;
  Serial.println("Orientation changed to: " + String(currentOrientation));
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
  while (analogRead(sensorPins[5]) < sensorOnLine[5])
    ;
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 255);
  analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 255);  // Speed (0-255)
  delay(100);
  forcePID(200);
}

void turnLeft() {
  //encoderRight = 0;
  //encoderLeft = 0;
  if (currentIntersection == 6 && currentOrientation == WEST) {
    Serial.println("I am turning left!");
    analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
    analogWrite(MOTORLEFT2, 70);    // Speed (0-255)
    analogWrite(MOTORRIGHT1, 100);  // HIGH = forward, change if reversed
    analogWrite(MOTORRIGHT2, 0);

    delay(200);
    while (analogRead(sensorPins[2]) < sensorOffLine[2])
      ;
    analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
    analogWrite(MOTORRIGHT2, 255);
    analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
    analogWrite(MOTORLEFT2, 255);  // Speed (0-255)
    delay(100);
    forcePID(200);
  } else {
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
    while (analogRead(sensorPins[0]) < sensorOnLine[0])
      ;
    analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
    analogWrite(MOTORRIGHT2, 255);
    analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
    analogWrite(MOTORLEFT2, 255);  // Speed (0-255)
    delay(100);
    forcePID(200);
  }
  currentOrientation = (currentOrientation + 1) % 4;
  Serial.println("Orientation changed to: " + String(currentOrientation));
}

void forcePID(int amountOfMillis) {
  long startTime = millis();
  while (millis() - startTime < amountOfMillis) {
    followLine();
  }
}

void noLineLogic() {
  Serial.println("Start of noLineLogic()");
  float timeSinceStart = millis();
  bool foundLine = false;
  driveMotors(215, 205);
  if (currentIntersection == specialIntersectionIndex) {
    switch (intersectionTurns[currentIntersection]) {
      case LEFT:
        if (DEBUG_TURNING) {
          Serial.println("I am turning left at noLine intersection");
        }
        delay(70);
        analogWrite(MOTORLEFT1, 70);    // HIGH = forward, change if reversed
        analogWrite(MOTORLEFT2, 0);     // Speed (0-255)
        analogWrite(MOTORRIGHT1, 220);  // HIGH = forward, change if reversed
        analogWrite(MOTORRIGHT2, 0);
        delay(900);
        currentOrientation = (currentOrientation + 1) % 4;

        break;
      case RIGHT:
        if (DEBUG_TURNING) {
          Serial.println("I am turning right at noLine intersection");
        }
        delay(200);
        analogWrite(MOTORLEFT1, 220);  // HIGH = forward, change if reversed
        analogWrite(MOTORLEFT2, 0);    // Speed (0-255)
        analogWrite(MOTORRIGHT1, 70);  // HIGH = forward, change if reversed
        analogWrite(MOTORRIGHT2, 0);
        delay(900);
        currentOrientation = (currentOrientation - 1 + 4) % 4;
        break;
    }
    baseSpeed = 50;
    forcePID(1000);
    baseSpeed = 210;
    currentIntersection++;
  } else {
    while (readDistance() > 13 && !foundLine) {
      if (millis() - timeSinceStart > 1200) {
        if (lineFinder()) {
          Serial.println("Found the line again!");
          foundLine = true;
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
      baseSpeed = 50;
      forcePID(1000);
      baseSpeed = 210;
      return;
    } else if (currentIntersection >= newPathSize && AStarActivated) {
      finalDance();
    } else {
      if (millis() - timeSinceStart <= 300) {
        uTurn();
      }
    }
  }
}
void uTurn() {
  currentOrientation = (currentOrientation + 2) % 4;
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
  analogWrite(MOTORLEFT2, 150);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 150);  // HIGH = forward, change if reversed
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
  analogWrite(MOTORLEFT1, 150);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);    // Speed (0-255)
  analogWrite(MOTORRIGHT1, 0);   // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 150);

  timeDelay = millis();
  while (millis() - timeDelay < 600) {
    for (int i = 0; i < 6; i++) {
      if (analogRead(sensorPins[i]) > sensorOnLine[i]) {
        return true;
      }
    }
  }
  //TURNING LEFT THIRD TIME
  analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 150);   // Speed (0-255)
  analogWrite(MOTORRIGHT1, 150);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);
  timeDelay = millis();
  while (millis() - timeDelay < 300) {
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
  for (int j = 0; j < 7; j++) {
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

int GetNeighbors(Node* node, Node* out[]) {
  int count = 0;

  if (node->northLimit) out[count++] = &MapNodes[node->y + 1][node->x];
  if (node->southLimit) out[count++] = &MapNodes[node->y - 1][node->x];
  if (node->westLimit) out[count++] = &MapNodes[node->y][node->x - 1];
  if (node->eastLimit) out[count++] = &MapNodes[node->y][node->x + 1];

  return count;
}

int AStar_Algorithm(Node* StartNode, Node* GoalNode, Node* path[]) {
  Node* open[MAX_NODES];
  Node* closed[MAX_NODES];
  int openSize = 0;
  int closedSize = 0;

  resetMap();
  Node* start = StartNode;

  start->g = 0;
  start->h = ManhattanDistance(StartNode->x, StartNode->y, GoalNode->x, GoalNode->y);
  start->parent = nullptr;
  open[openSize++] = start;

  // Problem found, it's related to how the push_back works
  // Is not saving Node* star and that leads to different values when tested
  /*Serial.print("h: ");
    Serial.print(start->h);
    Serial.print(" y: ");
    Serial.println(start->y);
    Serial.println(start->x);

    Serial.print("h: ");
    Serial.print(open[1]->h);
    Serial.print(" y: ");
    Serial.println(open[1]->y);
    Serial.println(open[1]->x);*/


  while (openSize > 0) {
    // Verify and choose the lowest f
    Node* current = open[0];
    int currentIndex = 0;
    /*
    Serial.print("x: ");
    Serial.print(current->x);
    Serial.print(" y: ");
    Serial.println(current->y);
*/
    for (int i = 1; i < openSize; i++) {
      if (open[i]->f() < current->f()) {
        current = open[i];
        currentIndex = i;
      }
    }

    // In case we are in the goal, find the parents of the nodes until get to the start node
    if (current->x == GoalNode->x && current->y == GoalNode->y) {
      int length = 0;

      while (current != nullptr) {
        path[length++] = current;
        current = current->parent;
        if (current) {
          Serial.print("x: ");
          Serial.print(current->x);
          Serial.print(" y: ");
          Serial.println(current->y);
        }
      }

      // Reverse the path to obtain the right order
      for (int i = 0; i < length / 2; i++) {
        Node* tmp = path[i];
        path[i] = path[length - 1 - i];
        path[length - 1 - i] = tmp;
      }

      return length;
    }

    // Move the current node to closed
    for (int i = currentIndex; i < openSize - 1; i++) open[i] = open[i + 1];
    openSize--;

    closed[closedSize++] = current;

    // Identify neighbors and explore them
    Node* neighbors[4];
    int nCount = GetNeighbors(current, neighbors);

    for (int i = 0; i < nCount; i++) {
      Node* neighbor = neighbors[i];
      bool skip = false;
      bool inOpen = false;

      // New current cost
      int gNew = current->g + 1;

      // Verify if the node is in the open array
      for (int j = 0; j < openSize; j++) {
        // If the current cost is greater or equal than the cost that Node already has,
        // move to the next neighbor
        if (open[j] == neighbor) {
          inOpen = true;
          // Update neighbor only if it finds a better path
          if (gNew < neighbor->g) {
            neighbor->g = gNew;
            neighbor->parent = current;
          }
          break;
        }
      }

      // Verify if the node is in the closed vector
      for (int j = 0; j < closedSize; j++) {
        // If the current cost is greater or equal than the cost that Node already has,
        // move to the next neighbor
        if (closed[j] == neighbor) {
          skip = true;
          // break;
        }
      }
      if (skip) continue;

      // In case the neighbor is neither in the open nor the closed vector,
      // add it to the open vector
      if (!inOpen) {
        neighbor->g = gNew;
        neighbor->h = ManhattanDistance(neighbor->x, neighbor->y, GoalNode->x, GoalNode->y);
        neighbor->parent = current;
        open[openSize++] = neighbor;
      }
    }
  }
  return 0;
}

// Functions to translate the Node Vector into positions and orders
importantVectors translateNodes2Orders(Node* Path[], int pathSize, int orientation) {
  importantVectors result;
  result.size = 0;

  int newOrientation = orientation;

  for (int i = 0; i < pathSize - 1; i++) {
    OrderResult r = getNextOrder(Path[i], Path[i + 1], newOrientation);
    newOrientation = r.newOrientation;

    int connections =
      Path[i]->northLimit + Path[i]->southLimit + Path[i]->eastLimit + Path[i]->westLimit;

    if (connections > 2) {
      result.nodes[result.size] = Path[i];
      result.orders[result.size] = r.order;

      Serial.print("result.nodes: ");
      Serial.print("x: ");
      Serial.print(result.nodes[result.size]->x);
      Serial.print(" y: ");
      Serial.println(result.nodes[result.size]->y);

      Serial.print("result.orders: ");
      Serial.println(result.orders[result.size]);


      result.size++;
    }
  }

  return result;
}

OrderResult getNextOrder(Node* currentNode, Node* nextNode, int originalOrientation) {
  int change_y = nextNode->y - currentNode->y;
  int change_x = nextNode->x - currentNode->x;
  int targetOrientation;
  if (change_y == 1) targetOrientation = NORTH;
  else if (change_y == -1) targetOrientation = SOUTH;
  else if (change_x == 1) targetOrientation = EAST;
  else if (change_x == -1) targetOrientation = WEST;
  int diff = (originalOrientation - targetOrientation + 4) % 4;
  OrderResult result;

  switch (diff) {
    case 0:
      result.order = FORWARD;
      break;
    case 1:
      result.order = RIGHT;
      break;
    case 3:
      result.order = LEFT;
      break;

    case 2:
      result.order = BACKWARD;
      break;
    default:
      Serial.println("Something happend in the order decision. Breaking.");
      break;
  }
  result.newOrientation = targetOrientation;
  return result;
}

ordersAndIndex Astar_robot(int currentIntersection, int distanceFromIntersection, int orientation) {
  // define 2 paths
  // path 1: from the last node to the goal node (distance = path + distanceFromIntersaction)
  // path 2: from the following node, to the (distance = path + (distanceBetweenIntersactions - distanceFromIntersaction))
  // choose the path with the lowest distance

  // specialNode will tell us if the robot will cross the node without dark line. If it doesn't cross that node, or if it does it and the order is to go forward, the number will stay at -1
  // Otherwise the number will corespond to the number of the order.

  int specialNode = -1;
  Node* path1[MAX_NODES];
  //Node* path2[MAX_NODES];
  Serial.println("Path 1 calculation");
  int size1 = AStar_Algorithm(intersectionNodes[currentIntersection], finalCylinderNode, path1);
  //Serial.println("Path 2 calculation");
  //int size2 = AStar_Algorithm(intersectionNodes[currentIntersection + 1], finalCylinderNode, path2);

  importantVectors myVectors;

  /*if (size1 > size2) {
    myVectors = translateNodes2Orders(path2, size2, orientation);
  } else {
    orientation = (orientation + 2) % 4;
    myVectors = translateNodes2Orders(path1, size1, orientation);
  }*/
  myVectors = translateNodes2Orders(path1, size1, orientation);

  ordersAndIndex out;
  out.size = 0;
  out.index = -1;

  for (int i = 0; i < myVectors.size; i++) {
    out.orders[out.size] = myVectors.orders[i];
    if (myVectors.nodes[i]->x == 4 && myVectors.nodes[i]->y == 4) {
      if (myVectors.orders[i] != FORWARD) out.index = out.size;
    }
    out.size++;
  }

  /*if (path1.size() <= path2.size()) {
      insertAtFront(finalOrders, BACKWARD);
    }*/

  out.orders[out.size++] = FORWARD;
  out.orders[out.size++] = LEFT;
  out.orders[out.size++] = LEFT;

  int n = out.size;
  Serial.println(n);

  // Testing if this will change the whole intersection array

  for (int i = 0; i < n; i++) {
    intersectionTurns[i] = out.orders[i];
    if (intersectionTurns[i] == FORWARD) Serial.println("forward");
    if (intersectionTurns[i] == BACKWARD) Serial.println("backward");
    if (intersectionTurns[i] == LEFT) Serial.println("left");
    if (intersectionTurns[i] == RIGHT) Serial.println("right");
  }
  Serial.println("Updated the intersectionTurns!");


  return out;
}

/*
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
}*/


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