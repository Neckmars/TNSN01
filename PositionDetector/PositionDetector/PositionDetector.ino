#include <math.h>
#define NMOTORS 2
#define RIGHT 0
#define LEFT 1

#define ON_LINE 1200

// --- Pin Setup - Right Wheel ---
#define PWMA_R 11    // PWM A pin to motor driver
#define PWMB_R 10    // PWM B pin to motor driver
#define encoderA_R 2    // Encoder channel A
#define encoderB_R 3      // Encoder channel B


// --- Pin Setup - Left Wheel ---
#define PWMA_L 6      // PWM pin to motor driver
#define PWMB_L 9      // PWM pin to motor driver
#define encoderA_L 4      // Encoder channel A
#define encoderB_L 5      // Encoder channel B

// --- Pin Setup - Line Sensors ---
// Left Sensors
#define LT_L3_PIN A0
#define LT_L2_PIN A1
#define LT_L1_PIN A2
// Right Sensors
#define LT_R1_PIN A3
#define LT_R2_PIN A4
#define LT_R3_PIN A5

/***************************   VARIABLES   ***************************/

int PPR = 3840;

// Position variables
double x = 0.0, y = 0.0, theta = 0.0;
double xPrev = 0.0, yPrev = 0.0, thetaPrev = 0.0;

// Velocities
// rad/s
float velAng[]  = {0.0, 0.0};
// RPM
float velTarget[] = {0.0, 0.0};
float velAngRPM[] = {0.0, 0.0};

// Encoder count
int velEnc[]       = {0, 0};
int velEncSlack[]  = {0, 0};

// Time variables - Estimate the position
float sampleT = 0.2;
long prevT = 0;
long time_prev, dt;

// PID control
float Kp = 0.3;
float Ki = 0.2;
float Kd = 0.2;

// In order that would represent physical location, needed to calculate weights based on horizontal position
const int sensorPins[6] = {
   LT_L3_PIN, LT_L2_PIN, LT_L1_PIN,
  LT_R1_PIN, LT_R2_PIN, LT_R3_PIN
};

const int MAX_CORRECTION = 55; 

const int baseSpeed = 200;


void setup() {
  Serial.begin(9600);

  pinMode(LT_L3_PIN, INPUT);
  pinMode(LT_L2_PIN, INPUT);
  pinMode(LT_L1_PIN, INPUT);
  pinMode(LT_R1_PIN, INPUT);
  pinMode(LT_R2_PIN, INPUT);
  pinMode(LT_R3_PIN, INPUT);

  pinMode(encoderA_R, INPUT);
  pinMode(encoderB_R, INPUT);
  pinMode(encoderA_L, INPUT);
  pinMode(encoderB_L, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderA_R), ENCODER1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_R), ENCODER1B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA_L), ENCODER2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_L), ENCODER2B_ISR, CHANGE);

  pinMode(MOTORRIGHT1, OUTPUT);
  pinMode(MOTORRIGHT2, OUTPUT);

  pinMode(MOTORLEFT1, OUTPUT);
  pinMode(MOTORLEFT2, OUTPUT);
  
  delay(2000);
  prevT = micros();
  time_prev = millis();

}

void loop() {

  int leftTurnSensor = analogRead(sensorPins[5]);
  int rightTurnSensor = analogRead(sensorPins[0]);
  
  if(leftTurnSensor > 600) {
    turnLeft();
  }
  else if(rightTurnSensor > 600) {
    turnRight();
  }
  else {
    int error = calulateWeightedError();
    int correction = calculatePID(error);

    int leftMotor = baseSpeed - correction;
    int rightMotor = baseSpeed + correction;

    driveMotors(leftMotor, rightMotor);
  }

  if(sampleT <= deltaT) {
    prevT = currT;
    noInterrupts();
    for(int i = 0; i < NMOTORS; i++){
      velAngRPM[i] = velEncSlack[i]/deltaT/PPR*60;
      velAng[i] = velAngRPM[i]*PI/30;
    }
    interrupts();

    // Print estimated pose
    Serial.print("X = ");
    Serial.print(x*1000);
    Serial.print(" (mm), ");
    Serial.print("Y = ");
    Serial.print(y*1000);
    Serial.print(" (mm), ");
    Serial.print("theta = ");
    Serial.print(theta*180/PI);
    Serial.println(" (deg)");
    Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%");
  }
}

void PositionEstimation(double deltaT) {
  /*
  Inputs:
    - deltaT: Time step.
  */

  double ct, st;
  ct = cos(theta); 
  st = sin(theta);

  x = xPrev + deltaT*((ct+st)*velAng[0] + (ct-st)*velAng[1])*R*ct/2;
  y = yPrev + deltaT*((st-ct)*velAng[0] + (st+ct)*velAng[1])*R*st/2;

  xPrev = x;
  yPrev = y;
  thetaPrev = theta;
}

int calulateWeightedError(){
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

    int weight = (i * 1000) - 2500; // calculate weight for the pin (-3500 -2500 -1500 -500 +500 +1500 +2500 +3500)
    weightedSum += (long)raw * weight; 
    // weightedSum += normalized * weight; // Using normalized readings
    sum += raw; // Sum all readings (so we can normalize, as we dont care about how dark/light just relation to each other)
  }

  // Will likely never happen (means completley lost line, and all white surface has 0 reflection) 
  // If it happens turn fully left or right
  if (sum == 0) { 
    return lastError > 0 ? 3500 : -3500;
  }

  return weightedSum / sum;
}

int calculatePID(int error){

  float e = (float)error;
  Serial.print("Error: ");
  Serial.println(e);
  
  integral += e;
  float derivative  = e - lastError;

  float correction = Kp * e /*+ Ki * integral*/ + Kd * derivative;
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  Serial.print("Correction: ");
  Serial.println(correction);
  return (int)correction;
}

void driveMotors(int left, int right){
    analogWrite(PWMA_L, left);  // HIGH = forward, change if reversed
    analogWrite(PWMB_L, 0);    // Speed (0–255)

  // RIGHT motor
    analogWrite(PWMA_R, right);  // HIGH = forward, change if reversed
    analogWrite(PWMA_R, 0);  

    Serial.print("Left motor: ");
    Serial.println(left);
    Serial.print("Right motor: ");
    Serial.println(right);


}

void turnRight(){
    static float travelRight = 0;
    static float travelLeft = 0;
    
    analogWrite(MOTORRIGHT1, 0);  // HIGH = forward, change if reversed
    analogWrite(MOTORRIGHT2, 150);  
    analogWrite(MOTORLEFT1, 255);  // HIGH = forward, change if reversed
    analogWrite(MOTORLEFT2, 0);    // Speed (0–255)
    
    while(travelLeft > 3840*1.25 && travelRight < -3840/2){

      travelRight += velEnc[0];
      travelLeft += velEnc[1];
      
      if(travelLeft > 3840*1.25){
        analogWrite(MOTORLEFT1, 0);
      }
      if(travelRight < -3840/2){
        analogWrite(MOTORRIGHT1, 0);
      }
    }
}

void turnLeft(){
    static float travelRight = 0;
    static float travelLeft = 0;

    analogWrite(MOTORLEFT1, 0);  // HIGH = forward, change if reversed
    analogWrite(MOTORLEFT2, 150);    // Speed (0–255)
    analogWrite(MOTORRIGHT1, 255);  // HIGH = forward, change if reversed
    analogWrite(MOTORRIGHT2, 0); 

    while(travelRight > 3840*1.25 && travelLeft < -3840/2){

      travelRight += velEnc[0];
      travelLeft += velEnc[1];
      
      if(travelRight > 3840*1.25){
        analogWrite(MOTORRIGHT1, 0);
      }
      if(travelLeft < -3840/2){
        analogWrite(MOTORLEFT1, 0);
      }
    }
}

template <int j>
void readEncoder() {
  // Function that counts each rising edge of a encoder
  velEnc[j]++;
}

// Decode direction from encoder
void ENCODER1A_ISR() {
  int stateA = digitalRead(ENCODER1A);
  int stateB = digitalRead(ENCODER1B);

  // If A leads B, forward; else reverse
  if (stateA == stateB) velEnc[0]++;
  else velEnc[0]--;
}

void ENCODER1B_ISR() {
  int stateA = digitalRead(ENCODER1A);
  int stateB = digitalRead(ENCODER1B);

  if (stateA != stateB) velEnc[0]++;
  else velEnc[0]--;
}

void ENCODER2A_ISR() {
  int stateA = digitalRead(ENCODER2A);
  int stateB = digitalRead(ENCODER2B);

  // If A leads B, forward; else reverse
  if (stateA == stateB) velEnc[1]++;
  else velEnc[1]--;
}

void ENCODER2B_ISR() {
  int stateA = digitalRead(ENCODER2A);
  int stateB = digitalRead(ENCODER2B);

  if (stateA != stateB) velEnc[1]++;
  else velEnc[1]--;
}
