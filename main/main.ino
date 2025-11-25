#include <Servo.h>

#define LT_L3_PIN A0
#define LT_L2_PIN A1
#define LT_L1_PIN A2
#define LT_R1_PIN A3
// Middle of line
#define LT_R2_PIN A4
#define LT_R3_PIN A5

#define MOTORLEFT1 6
#define MOTORLEFT2 9
#define MOTORRIGHT1 11
#define MOTORRIGHT2 10

#define ENCODER1A 2
#define ENCODER1B 3
#define ENCODER2A 4
#define ENCODER2B 5

#define DEBUG_SENSORS 1
#define DEBUG_DISTANCE_SENSOR 1

//#define ON_LINE 1200 // TODO: Value representing a reading ontop of the line

#define gripperServoPin 1000   //TODO: placehodler
#define verticalServoPin 1000  //TODO: placehodler
#define trigPin = 1000;
#define echoPin = 1000;

// In order that would represent physical location, needed to calculate weights based on horizontal position
const int sensorPins[6] = {
  LT_L3_PIN, LT_L2_PIN, LT_L1_PIN,
  LT_R1_PIN, LT_R2_PIN, LT_R3_PIN
};

int sensorMin[6];
int sensorMax[6];
int normalized[6];

int lastError;
int integral;

float Kp = 0.3;
float Ki = 0.2;
float Kd = 0.2;

const int MAX_CORRECTION = 55;

const int baseSpeed = 200;

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;

//gripper variables
Servo gripperServo;
Servo verticalServo;

const int DISTANCE_FROM_GRIPPER = 60;
const int GRIPPER_CLOSED_POS = 10;
const int GRIPPER_OPEN_POS = 55;
const int GRIPPER_DROPOFF_POS = 15;
const int VERTICAL_DOWN_POS = 0;
const int GRIPPER_UP_POS = 100;

void gripperSetup() {
  gripperServo.attatch(gripperPin);
  verticalServo.attach(verticalPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  gripperServo.write(GRIPPER_OPEN_POS);
  verticalServo.write(VERTICAL_DOWN_POS);
}

void lineFollowingSetup() {
  pinMode(LT_L3_PIN, INPUT);
  pinMode(LT_L2_PIN, INPUT);
  pinMode(LT_L1_PIN, INPUT);
  pinMode(LT_R1_PIN, INPUT);
  pinMode(LT_R2_PIN, INPUT);
  pinMode(LT_R3_PIN, INPUT);

  pinMode(MOTORLEFT1, OUTPUT);
  pinMode(MOTORLEFT2, OUTPUT);
  pinMode(MOTORRIGHT1, OUTPUT);
  pinMode(MOTORRIGHT2, OUTPUT);

  pinMode(ENCODER1A, INPUT_PULLUP);
  pinMode(ENCODER1B, INPUT_PULLUP);
  pinMode(ENCODER2A, INPUT_PULLUP);
  pinMode(ENCODER2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER1A), ENCODER1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1B), ENCODER1B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2A), ENCODER2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2B), ENCODER2B_ISR, CHANGE);
}

long getDistanceSensor() {
  long duration, distance;
  // Read distance
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = microsecondsToCentimeters(duration);
  if (DEBUG_DISTANCE_SENSOR) {
    Serial.print(distance);
    Serial.println(" cm");
  }
  return distance;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

void pickUpAndStore(){
  gripperServo.write(GRIPPER_CLOSED_POS);
  delay(1000);
  verticalServo.write(GRIPPER_UP_POS);
  delay(1000);
  gripperServo.write(GRIPPER_DROPOFF_POS);
  delay(1000);
  gripperServo.write(GRIPPER_OPEN_POS);
  verticalServo.write(VERTICAL_DOWN_POS);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  gripperSetup();
  lineFollowingSetup();
}

void calibrateSensors() {
  Serial.println("Calibraiting");

  for (int i = 0; i < 6; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  for (int t = 0; t < 400; t++) {
    // Rotate bot left ~90, right ~180, left~90
    if (t < 100 || t > 300) {
      // rotate left
      analogWrite(MOTORLEFT1, 0);
      analogWrite(MOTORLEFT2, 50);
      analogWrite(MOTORRIGHT1, 50);
      analogWrite(MOTORRIGHT2, 0);
    } else {
      // rotate right
      analogWrite(MOTORLEFT1, 0);
      analogWrite(MOTORLEFT2, 50);
      analogWrite(MOTORRIGHT1, 50);
      analogWrite(MOTORRIGHT2, 0);
    }

    // Read sensors while sweeping the line
    for (int i = 0; i < 6; i++) {
      int val = analogRead(sensorPins[i]);

      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    delay(5);
  }

  driveMotors(0, 0);
  Serial.println("Calibraiting done");
}

void loop() {
  int leftTurnSensor = analogRead(sensorPins[5]);
  int rightTurnSensor = analogRead(sensorPins[0]);

  if (leftTurnSensor > 600) {
    turnLeft();
  } else if (rightTurnSensor > 600) {
    turnRight();
  } else {

    int error = calulateWeightedError();
    int correction = calculatePID(error);

    int leftMotor = baseSpeed - correction;
    int rightMotor = baseSpeed + correction;

    driveMotors(leftMotor, rightMotor);
  }
}

void driveMotors(int left, int right) {
  analogWrite(MOTORLEFT1, left);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);     // Speed (0–255)

  // RIGHT motor
  analogWrite(MOTORRIGHT1, right);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  Serial.print("Left motor: ");
  Serial.println(left);
  Serial.print("Right motor: ");
  Serial.println(right);
}

int calulateWeightedError() {
  long weightedSum = 0;
  long sum = 0;

  for (int i = 1; i < 5; i++) {
    int raw = analogRead(sensorPins[i]);

    if (DEBUG_SENSORS) {
      Serial.print("Raw pin ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(raw);
    }

    raw = constrain(raw, sensorMin[i], sensorMax[i]);
    int normalized = map(raw, sensorMin[i], sensorMax[i], 0, 1000);

    int weight = (i * 1000) - 2500;  // calculate weight for the pin (-3500 -2500 -1500 -500 +500 +1500 +2500 +3500)
    // weightedSum += (long)raw * weight;
    weightedSum += (long)normalized * weight;  // Using normalized readings
    sum += raw;                                // Sum all readings (so we can normalize, as we dont care about how dark/light just relation to each other)
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
  Serial.print("Error: ");
  Serial.println(e);

  integral += e;
  float derivative = e - lastError;

  float correction = Kp * e /*+ Ki * integral*/ + Kd * derivative;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  Serial.print("Correction: ");
  Serial.println(correction);
  return (int)correction;
}

void turnRight() {
  static float travelRight = 0;
  static float travelLeft = 0;

  analogWrite(MOTORRIGHT1, 0);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 150);
  analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);    // Speed (0–255)

  while (travelLeft > 3840 * 1.25 && travelRight < -3840 / 2) {

    travelRight += encoderCount1;
    travelLeft += encoderCount2;

    if (travelLeft > 3840 * 1.25) {
      analogWrite(MOTORLEFT1, 0);
    }
    if (travelRight < -3840 / 2) {
      analogWrite(MOTORRIGHT1, 0);
    }
  }
}

void turnLeft() {
  static float travelRight = 0;
  static float travelLeft = 0;

  analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 150);   // Speed (0–255)
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  while (travelRight > 3840 * 1.25 && travelLeft < -3840 / 2) {

    travelRight += encoderCount1;
    travelLeft += encoderCount2;

    if (travelRight > 3840 * 1.25) {
      analogWrite(MOTORRIGHT1, 0);
    }
    if (travelLeft < -3840 / 2) {
      analogWrite(MOTORLEFT1, 0);
    }
  }
}

// Decode direction from encoder
void ENCODER1A_ISR() {
  int stateA = digitalRead(ENCODER1A);
  int stateB = digitalRead(ENCODER1B);

  // If A leads B, forward; else reverse
  if (stateA == stateB) encoderCount1++;
  else encoderCount1--;
}

void ENCODER1B_ISR() {
  int stateA = digitalRead(ENCODER1A);
  int stateB = digitalRead(ENCODER1B);

  if (stateA != stateB) encoderCount1++;
  else encoderCount1--;
}

void ENCODER2A_ISR() {
  int stateA = digitalRead(ENCODER2A);
  int stateB = digitalRead(ENCODER2B);

  // If A leads B, forward; else reverse
  if (stateA == stateB) encoderCount2++;
  else encoderCount2--;
}

void ENCODER2B_ISR() {
  int stateA = digitalRead(ENCODER2A);
  int stateB = digitalRead(ENCODER2B);

  if (stateA != stateB) encoderCount2++;
  else encoderCount2--;
}
