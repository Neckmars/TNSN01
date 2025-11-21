// --- Pin Setup ---
const int motorPWM1a = 6;      // PWM pin to motor driver
const int motorPWM1b = 9;      // Direction pin to motor driver
const int motorPWM2a = 10;      // PWM pin to motor driver
const int motorPWM2b = 11;      // Direction pin to motor driver

const int encoder1A = 2;      // Encoder channel A (must be interrupt-capable)
const int encoder1B = 3;      // Encoder channel B (interrupt-capable)
const int encoder2A = 4;      // Encoder channel A (must be interrupt-capable)
const int encoder2B = 5;      // Encoder channel B (interrupt-capable)

int PWMspeed = 20; 

// Encoder count
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;

// Decode direction from encoder
void encoder1A_ISR() {
  int stateA = digitalRead(encoder1A);
  int stateB = digitalRead(encoder1B);

  // If A leads B, forward; else reverse
  if (stateA == stateB) encoderCount1++;
  else encoderCount1--;
}

void encoder1B_ISR() {
  int stateA = digitalRead(encoder1A);
  int stateB = digitalRead(encoder1B);

  if (stateA != stateB) encoderCount1++;
  else encoderCount1--;
}

void encoder2A_ISR() {
  int stateA = digitalRead(encoder2A);
  int stateB = digitalRead(encoder2B);

  // If A leads B, forward; else reverse
  if (stateA == stateB) encoderCount2++;
  else encoderCount2--;
}

void encoder2B_ISR() {
  int stateA = digitalRead(encoder2A);
  int stateB = digitalRead(encoder2B);

  if (stateA != stateB) encoderCount2++;
  else encoderCount2--;
}

/*void setup() {
  Serial.begin(9600);

  pinMode(motorPWM1a, OUTPUT);
  pinMode(motorPWM1b, OUTPUT);
  pinMode(motorPWM2a, OUTPUT);
  pinMode(motorPWM2b, OUTPUT);

  pinMode(encoder1A, INPUT_PULLUP);
  pinMode(encoder1B, INPUT_PULLUP);
  pinMode(encoder2A, INPUT_PULLUP);
  pinMode(encoder2B, INPUT_PULLUP);
  

  attachInterrupt(digitalPinToInterrupt(encoder1A), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1B), encoder1B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2A), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2B), encoder2B_ISR, CHANGE);

  // Start motor forward
  analogWrite(motorPWM1b, 0);  // HIGH = forward, change if reversed
  analogWrite(motorPWM1a, 50);    // Speed (0–255)
  analogWrite(motorPWM2b, 0);  // HIGH = forward, change if reversed
  analogWrite(motorPWM2a, 50); 
}

void loop() {
  static long lastCount = 0;

  // Print only if count changed (keeps Serial clean)
  if (encoderCount1 != lastCount) {
    Serial.print("Encoder Count 1: ");
    Serial.println(encoderCount1/3840);
    Serial.print("Encoder Count 2: ");
    Serial.println(encoderCount2/3840);
    lastCount = encoderCount1;
  }

  // Optional: after some time, reverse direction
  if (millis() > PWMspeed*100 && PWMspeed < 255) {
    analogWrite(motorPWM1a, PWMspeed);    // Speed (0–255)
    PWMspeed = PWMspeed + 20; 
  }
}*/