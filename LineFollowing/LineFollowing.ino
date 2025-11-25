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

#define ENCODER_RIGHT_A 2
#define ENCODER_RIGHT_B 3
#define ENCODER_LEFT_A 0
#define ENCODER_LEFT_B 1

#define STARTBUTTON 13

#define ON_LINE 1200  // TODO: Value representing a reading ontop of the line

// In order that would represent physical location, needed to calculate weights based on horizontal position
const int sensorPins[6] = {
  LT_L3_PIN, LT_L2_PIN, LT_L1_PIN,
  LT_R1_PIN, LT_R2_PIN, LT_R3_PIN
};

int lastError;
int integral;

float Kp = 0.3;
float Ki = 0.0002;
float Kd = 0.2;


const int MAX_CORRECTION = 55;

const int baseSpeed = 120;

volatile long encoderRight = 0;
volatile long encoderLeft = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

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

  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), ENCODER_RIGHT_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), ENCODER_RIGHT_B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), ENCODER_LEFT_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), ENCODER_LEFT_B_ISR, CHANGE);

  pinMode(STARTBUTTON, INPUT_PULLUP);

  while (digitalRead(STARTBUTTON) == HIGH);
}

void loop() {
  //TODO: Might be good to normalize sensor readings, would require a calibration step where the robot sweeps over the line to see highest and lowest reading for each sensor.
  Serial.println(encoderLeft);
  Serial.println(encoderRight);
  int leftTurnSensor = analogRead(sensorPins[0]);
  int rightTurnSensor = analogRead(sensorPins[5]);

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

/*int calculateError(){
  int L1 = analogRead(LT_L1_PIN);
  int L2 = analogRead(LT_L2_PIN);
  int L3 = analogRead(LT_L3_PIN);
  int L4 = analogRead(LT_L4_PIN);

  int R1 = analogRead(LT_R1_PIN);
  int R2 = analogRead(LT_R2_PIN);
  int R3 = analogRead(LT_R3_PIN);
  int R4 = analogRead(LT_R4_PIN);

  Serial.print("L1: "); 
  Serial.print(L1);
  Serial.print("  L2: "); 
  Serial.print(L2);
  Serial.print("  L3: "); 
  Serial.print(L3);
  Serial.print("  L4: "); 
  Serial.print(L4);
  Serial.print("  R1: "); 
  Serial.print(R1);
  Serial.print("  R2: "); 
  Serial.print(R2);
  Serial.print("  R3: "); 
  Serial.print(R3);
  Serial.print("  R4: "); 
  Serial.println(R4);

  int leftSum = L1 + L2 + L3 + L4;
  int rightSum = R1 + R2 + R3 + R4;

  // error = 0 -> on line
  // error > 0 -> tilted right (need to turn left)
  // error < 0 -> tilted left (need to turn right)
  int error = leftSum - rightSum;
}*/

int calulateWeightedError() {
  long weightedSum = 0;
  long sum = 0;

  for (int i = 1; i < 5; i++) {
    int raw = analogRead(sensorPins[i]);
    Serial.print("Raw pin ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(raw);
    // raw = constrain(raw, sensorMin[i], sensorMax[i]);
    // int normalized = map(raw, sensorMin[i], sensorMax[i], 0, 1000);

    int weight = (i * 1000) - 2500;  // calculate weight for the pin (-3500 -2500 -1500 -500 +500 +1500 +2500 +3500)
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
  encoderRight = 0;
  encoderLeft = 0;

  Serial.println("I am turning right!");
  analogWrite(MOTORRIGHT1, 0);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 150);
  analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 0);    // Speed (0–255)

  while (encoderRight < 3840 * 1.25 || encoderLeft > -3840 / 2) {


    if (encoderLeft > 3840 * 1.25) {
      analogWrite(MOTORLEFT1, 0);
    }
    if (encoderRight < -3840 / 2) {
      analogWrite(MOTORRIGHT2, 0);
    }
  }
}

void turnLeft() {
  encoderRight = 0;
  encoderLeft = 0;
   Serial.println("I am turning left!");


  analogWrite(MOTORLEFT1, 0);     // HIGH = forward, change if reversed
  analogWrite(MOTORLEFT2, 150);   // Speed (0–255)
  analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
  analogWrite(MOTORRIGHT2, 0);

  while (encoderLeft < 3840 * 1.25 || encoderRight > -3840 / 2) {

    if (encoderRight > 3840 * 1.25) {
      analogWrite(MOTORRIGHT1, 0);
    }
    if (encoderLeft < -3840 / 2) {
      analogWrite(MOTORLEFT2, 0);
    }
  }
}

// Decode direction from encoder
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
