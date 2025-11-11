#include <math.h>
#define NMOTORS 2
#define RIGHT 0
#define LEFT 1

// --- Pin Setup - Right Wheel ---
#define PWMA_R 6    // PWM A pin to motor driver
#define PWMB_R 9     // PWM B pin to motor driver
#define encoderA_R 2    // Encoder channel A
#define encoderB_R 3      // Encoder channel B


// --- Pin Setup - Left Wheel ---
#define PWMA_L 10      // PWM pin to motor driver
#define PWMB_L 11      // PWM pin to motor driver
#define encoderA_L 4      // Encoder channel A
#define encoderB_L 5      // Encoder channel B


/***************************   VARIABLES   ***************************/
int PWMspeed = 20;

// Robot dimentions
const double a_b = 0.075;
const double R = 0.035;
// 7.5 cm x 2

// Position variables
double x = 0.0, y = 0.0, theta = 0.0;
double xPrev = 0.0, yPrev = 0.0, thetaPrev = 0.0;

// Velocities
// rad/s
float velAng[]  = {0.0, 0.0};
// RPM
float velTarget[] = {0.0, 0.0};
float velAngRPM[] = {0.0, 0.0};
// pwr
int pwr[] = {0, 0};

// Global velocities
double gvel_X = 0.0, gvel_y = 0.0;
// Robot-Reference velocity
double vx = 0.0, vw = 0.0;

// Encoder count
int velEnc[]       = {0, 0};
int velEncSlack[]  = {0, 0};

// Goal
float xGoal[] = {0.0, 0.3, 1.0, 1.0};
float yGoal[] = {0.0, 0.0, 1.0, 2.0};
int seq = 1;
int maxSeq = 3;

// Error 
double errorTheta = PI/2;

// Motor data
const int PPR = 1390;
float eprev[] = {0.0, 0.0};
float eintegral[] = {0.0, 0.0};

// Time variables
float sampleT = 0.1;
long prevT = 0;
long time_prev, dt;

void setup() {
  Serial.begin(9600);

  pinMode(encoderA_R, INPUT);
  pinMode(encoderB_R, INPUT);
  pinMode(encoderA_L, INPUT);
  pinMode(encoderB_L, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderA_R), readEncoder<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA_L), readEncoder<1>, CHANGE);
  
  pinMode(PWMA_R, OUTPUT);
  pinMode(PWMB_R, OUTPUT);

  pinMode(PWMA_L, OUTPUT);
  pinMode(PWMB_L, OUTPUT);
  
  delay(2000);
  prevT = micros();
  time_prev = millis();

}

void loop() {
  // Time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );

  // Delta time to calculate the angle direction
  dt = millis()-time_prev;
  time_prev= millis();
  //SetMotor(60, PWMA_R, PWMB_R);
  //SetMotor(60, PWMA_L, PWMB_L);

  if (theta >= PI){
    theta -= 2*PI;
  }
  else if (theta <= -PI){
    theta += 2*PI;
  }

  if (sampleT <= deltaT) {
    prevT = currT;
    noInterrupts();
    for(int i = 0; i < NMOTORS; i++){
      // Reset counter
      velEncSlack[i] = velEnc[i];
      velEnc[i] = 0;
    }
    interrupts();

    for(int i = 0; i < NMOTORS; i++){
      velAngRPM[i] = velEncSlack[i]/deltaT/PPR*60;
      velAng[i] = velAngRPM[i]*PI/30;
    }
    PositionEstimation(deltaT);

    if (seq < maxSeq) {
      CalculatePositionError(xGoal[seq], yGoal[seq], vx, vw);
    }
    else {
      StopMotors();
    }

    CalculateVelAng(vx, vw);

    RPMtoPWM(velAng[RIGHT], velTarget[RIGHT], deltaT, pwr[RIGHT], RIGHT);
    RPMtoPWM(velAng[LEFT], velTarget[LEFT], deltaT, pwr[LEFT], LEFT);
    //SetMotor(pwr[RIGHT], PWMA_R, PWMB_R);
    //SetMotor(pwr[LEFT], PWMA_L, PWMB_L);
    SetMotor(120, PWMA_R, PWMB_R);
    SetMotor(120, PWMA_L, PWMB_L);

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

  // Logic to change sequence
  double errorX = xGoal[seq] - x;
  double errorY = yGoal[seq] - y;
  if((sqrt(errorX*errorX + errorY*errorY) < 0.03)){
    StopMotors();
    delay(100);
  }
}

void CalculatePositionError(double xGoal_l, double yGoal_l, double &vx, double &vw){
  // Position error
  double errorX = xGoal_l - x;
  double errorY = yGoal_l - y;
  
  //kp_pos is the proportional constant error in the position 
  double kp_pos=1.6;
  double vx_global = kp_pos * errorX;
  double vy_global = kp_pos * errorY;
  
  double ct = cos(theta); 
  double st = sin(theta);

  vw = 0;
  vx = vx_global*ct + st * vy_global;

}

void CalculateOrientationError(double thetaGoal_l, double &vel, double &vw, double &errorTheta){
  // Orientation error
  errorTheta = thetaGoal_l - theta;
  if (errorTheta >= PI){
    errorTheta -= 2*PI;
  }
  else if (errorTheta <= -PI){
    errorTheta += 2*PI;
  }
  // No velocity
  vel = 0;
  //kp_ang is the proportional constant error in the position 
  double kp_ang = 0.8;
  vw = kp_ang * errorTheta;
  // Logic in case that vw is greater than PI/5 or lesser than 0.4 
  // Maybe can remove this
  /*if (fabs(vw) > PI/5){
      vw = (vw/fabs(vw))*PI/5;
  }
  if (fabs(errorTheta)<0.4){
      vw = (errorTheta/fabs(errorTheta))*PI/5.3;
  }*/
}

void CalculateVelAng(double vx, double vw) { 
  /*
  Function that computes the velocity in rpm and the direction 
  of each wheel from the absolute velocity.

  Inputs:
    - vx: Linear velocity in X axis, in m/s.
    - vw: Angular velocity in Z axis, in rad/s.
  */
  double w_R, w_L;

  // Angular velocity of each motor in rad/s
  w_R = (vx + vw * a_b) / R;
  w_L = (vx - vw * a_b) / R;

  // Angular velocity of each motor in RPM, to target.
  velTarget[RIGHT] = w_R*30/PI;
  velTarget[LEFT]  = w_L*30/PI;
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

void RPMtoPWM(float value, float target, float deltaT, int &pwr, int DWheel) {
  float kp = 0.5, kd = 0.02, ki = 6.0, vmin = 10.0;
  int umax = 255;

  float e, dedt;
  e = (target - value)*((float)fabs(target) > vmin);
  dedt = (e - eprev[DWheel] )/(deltaT)*((float) fabs(target) > vmin);
  eintegral[DWheel] = (eintegral[DWheel] + e * deltaT)*((float) fabs(target) > vmin);
  
  pwr = (int) (kp * e + kd * dedt + ki * eintegral[DWheel]);
  eprev[DWheel] = e;

}


template <int j>
void readEncoder() {
  // Function that counts each rising edge of a encoder
  velEnc[j]++;
}

void SetMotor(int pwmVal, int pinPWM_A, int pinPWM_B) {
  /* 
  Function to setup pins to control motors.
  Inputs:
    - pwmVal    : PWM speed value, between -255 to 255
    - pinPWM_A  : Pin A
    - pinPWM_B  : Pin B
  */

  if (pwmVal >= 0) {
    analogWrite(pinPWM_A, pwmVal);
    analogWrite(pinPWM_B, 0);
  }
  else {
    analogWrite(pinPWM_A, 0);
    analogWrite(pinPWM_B, -pwmVal);
  }

}

void StopMotors(){
  // Function that stops each DC motor. 
  CalculateVelAng(0,0);
  
  SetMotor(0.0, PWMA_R, PWMB_R);
  SetMotor(0.0, PWMA_L, PWMB_L);

  CalculateVelAng(0,0);
  delay(6000);
}