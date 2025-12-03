#include <Servo.h>

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

#define TRIGPIN 8
#define ECHOPIN 7

#define GRIPPERPIN 9
#define VERTICALPIN 10

#define GRIPPER_CLOSED_POS 10
#define GRIPPER_OPEN_POS 60
#define GRIPPER_DROPOFF_POS 30
#define VERTICAL_DOWN_POS 175
#define VERTICAL_UP_POS 65
#define VERTICAL_DRIVE_POS 85
#define GRIPPER_UP_POS 30
#define DISTANCE_FROM_GRIPPER 9

#define STARTBUTTON 12

#define ON_LINE 850  // TODO: Value representing a reading ontop of the line

#define DEBUG_PID 0
#define DEBUG_LINE_SENSORS 0
#define DEBUG_TURN_SENSORS 0
#define DEBUG_DISTANCE_SENSORS 1

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

float Kp = 0.25;
float Ki = 0.0002;
float Kd = 0;


const int MAX_CORRECTION = 55;

const int baseSpeed = 200;

volatile long encoderRight = 0;
volatile long encoderLeft = 0;

long lastTime = millis();

long distance = 0;

enum direction {
  FORWARD,
  LEFT,
  RIGHT,
  BACKWARD
};

int currentIntersection = 0;
direction intersectionTurns[14] = { LEFT, LEFT, LEFT, FORWARD, LEFT, /*no line - keep going forward,*/ RIGHT, LEFT, FORWARD, LEFT /*Is now in final dead end*/, FORWARD, FORWARD, FORWARD, LEFT, LEFT };

//TODO: need to detect forward + left XOR right (-> T ->) crossing (would currently turn, and never go forward)
// Option 1:
// Following line -> at least one turn sensor goes high -> note the sensors that went high, but continue forward until they go low again -> note if forward is still high -> we now know what the intersection looks like, choose correct from possible options
// we should only have slightly overshot the intersection so going left, right or forward is still possible.

// Option 2:
// Treat all turns, including simple left and right turns as intersections, and hard code the right choice for each one.
// Result: Detect right or left turn -> count it as an intersection so look for correct turn in the hard coded path.
// Means hard coding every single turn, but eliminates problem of -> T -> crossings

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
  gripperServo.write(GRIPPER_OPEN_POS);
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
}

void loop() {
  //TODO: Might be good to normalize sensor readings, would require a calibration step where the robot sweeps over the line to see highest and lowest reading for each sensor.
  ////Serial.println(encoderLeft);
  ////Serial.println(encoderRight);
  long sum = 0;
  if(DEBUG_DISTANCE_SENSORS && millis() - lastTime > 20){
    distance = readDistance();
    Serial.println("Distance: " + String(distance));
    lastTime = millis();
  }
  for (int i = 1; i < 5; i++) {
    int raw = analogRead(sensorPins[i]);
    sum += raw;
  }
  if (sum < 800) {
    //Serial.println("sum: " + String(sum));
    noLineLogic();
  }

  int leftTurnSensor = analogRead(sensorPins[0]);
  int rightTurnSensor = analogRead(sensorPins[5]);
  // Read distance

  if (distance <= DISTANCE_FROM_GRIPPER) {  // Might be inacurate when close range in practice so might have to do it "blind", i.e move forward x amount, then do closing
    Serial.println("Object too close, grabbing it!");
    pickUpAndStore();
  }



  if (DEBUG_TURN_SENSORS) {
    Serial.println("Left turn sensor: " + String(leftTurnSensor));
    Serial.println("Right turn sensor: " + String(rightTurnSensor));
  }

  if (leftTurnSensor > ON_LINE) {
    hasLeftTurn = 1;
  }
  if (rightTurnSensor > ON_LINE) {
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
      // We overshot the intersection, check for forward option
      if (analogRead(sensorPins[2]) > ON_LINE || analogRead(sensorPins[3]) > ON_LINE) {
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
          Serial.println("Switch case turning left");
          turnLeft();
          break;
        case RIGHT:
          Serial.println("Switch case turning right");
          turnRight();
          break;
        case FORWARD:
          Serial.println("Switch case FORWARD");
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

  // Follow line PID
  followLine();
}

void followLine() {
  int error = calulateWeightedError();
  int correction = calculatePID(error);

  int leftMotor = baseSpeed + correction;
  int rightMotor = baseSpeed - correction;

  driveMotors(leftMotor, rightMotor);
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
                                 // The ping travels out and back, so to find the distance of the object we
                                 // take half of the distance travelled.
}

void pickUpAndStore() {
  analogWrite(MOTORLEFT1, 0);   // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);   // Speed (0–255)
  analogWrite(MOTORRIGHT1, 0);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);
  verticalServo.attach(VERTICALPIN);
  gripperServo.attach(GRIPPERPIN);
  //Serial.println("Vertical down position!");
  delay(100);
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
  gripperServo.write(GRIPPER_OPEN_POS);

  verticalServo.detach();
  gripperServo.detach();
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
  analogWrite(MOTORLEFT2, 0);     // Speed (0–255)

  // RIGHT motor
  analogWrite(MOTORRIGHT1, right);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  //Serial.print("Left motor: ");
  //Serial.println(left);
  //Serial.print("Right motor: ");
  //Serial.println(right);
}

int calulateWeightedError() {
  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < 6; i++) {
    int raw = analogRead(sensorPins[i]);
    if (DEBUG_LINE_SENSORS) {
      Serial.print("Raw pin ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(raw);
    }

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
  //Serial.print("Error: ");
  //Serial.println(e);

  // integral += e;
  float derivative = e - lastError;
  lastError = e;

  float correction = Kp * e /*+ Ki * integral*/ + Kd * derivative;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  //Serial.print("Correction: ");
  //Serial.println(correction);
  if (DEBUG_PID) {
    Serial.print("P-term: ");
    Serial.println(String(e * Kp));
    Serial.print("D-term: ");
    Serial.println(Kd * derivative);
    Serial.println("Correction: " + String(correction));
  }
  return (int)correction;
}

void turnRight() {
  // encoderRight = 0;
  // encoderLeft = 0;
  delay(150);
  Serial.println("I am turning right!");
  analogWrite(MOTORRIGHT1, 0);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 255);
  analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);    // Speed (0–255)

  delay(200);
  while (analogRead(sensorPins[6]) < ON_LINE);
  analogWrite(MOTORLEFT1, 255);   // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 255);   // Speed (0–255)
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 255);
  forcePID();
}

void turnLeft() {
  //encoderRight = 0;
  //encoderLeft = 0;
  delay(150);
  Serial.println("I am turning left!");
  analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 255);     // Speed (0–255)
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);
  
  delay(200);
  while (analogRead(sensorPins[0]) < ON_LINE);
  analogWrite(MOTORLEFT1, 255);   // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 255);   // Speed (0–255)
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 255);

  forcePID();
}

void forcePID(){
  for (int i = 0; i < 400; i++) {
    followLine();
  }
}

void noLineLogic() {
  bool foundLine = false;
  driveMotors(170,160);
  while (distance > 15 && !foundLine) {
    for (int i = 0; i < 6; i++) {
      if (analogRead(sensorPins[i]) > ON_LINE) {
        foundLine = true;
        break;
      }
    }
    delay(10);
  }
  if (foundLine) {
    forcePID();
    return;
  }else if(currentIntersection == 5){
    Serial.println("I am turning left at intersection 5!");

  analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 255);     // Speed (0–255)
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  delay(400);
  
  } 
  else {
    uTurn();
  }
}

void uTurn() {
  Serial.println("I am doing a u-turn!!");
  analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 255);   // Speed (0–255)
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  delay(200);
  while (analogRead(sensorPins[0]) < ON_LINE);

  analogWrite(MOTORLEFT1, 255);   // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 255);   // Speed (0–255)
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 255);
  delay(100);
  forcePID();
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
