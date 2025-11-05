// --- Pin Setup ---
const int motorPWMa = 5;      // PWM pin to motor driver
const int motorPWMb = 6;      // Direction pin to motor driver

const int encoderA = 2;      // Encoder channel A (must be interrupt-capable)
const int encoderB = 3;      // Encoder channel B (interrupt-capable)

int PWMspeed = 20; 

// Encoder count
volatile long encoderCount = 0;

// Decode direction from encoder
void encoderA_ISR() {
  int stateA = digitalRead(encoderA);
  int stateB = digitalRead(encoderB);

  // If A leads B, forward; else reverse
  if (stateA == stateB) encoderCount++;
  else encoderCount--;
}

void encoderB_ISR() {
  int stateA = digitalRead(encoderA);
  int stateB = digitalRead(encoderB);

  if (stateA != stateB) encoderCount++;
  else encoderCount--;
}

void setup() {
  Serial.begin(9600);

  pinMode(motorPWMa, OUTPUT);
  pinMode(motorPWMb, OUTPUT);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderB_ISR, CHANGE);

  // Start motor forward
  analogWrite(motorPWMb, 0);  // HIGH = forward, change if reversed
  analogWrite(motorPWMa, 50);    // Speed (0–255)
}

void loop() {
  static long lastCount = 0;

  // Print only if count changed (keeps Serial clean)
  if (encoderCount != lastCount) {
    Serial.print("Encoder Count: ");
    Serial.println(encoderCount/3840);
    lastCount = encoderCount;
  }

  // Optional: after some time, reverse direction
  if (millis() > PWMspeed*100) {
    analogWrite(motorPWMa, PWMspeed);    // Speed (0–255)
    PWMspeed = PWMspeed + 20; 
  }
}