// PID Line Follower for Arduino Leonardo + QTR-8A (Analog)
// Uses Pololu QTRSensorsAnalog library
// Reads line position (0..7000) and applies PID to motor speeds

#include <QTRSensors.h>

// ========== CONFIG ==========
// Sensors
QTRSensors qtr;

const uint8_t SensorCount = 3;
uint16_t sensorValues[SensorCount];

// PID parameters (start values — tune below)
double Kp = 0.40;   // proportional gain
double Ki = 0.0008; // integral gain
double Kd = 3.2;    // derivative gain

// PID / timing
unsigned long lastTime = 0;
const unsigned long sampleTime = 25;  // ms between PID updates
double integral = 0;
double lastError = 0;
double integralMax = 3000;  // clamp integral (prevent windup)

// Motor pins (example: L298/TB6612 style)
const int leftIn1Pin   = 6;   // direction
const int leftIn2Pin   = 9;
const int rightIn1Pin  = 11;   // direction
const int rightIn2Pin  = 10;

// Motion / speed limits
const int PWM_MAX = 255;
const int PWM_MIN = 0;
int baseSpeed = 120; // nominal forward speed (tweak 0..255)

// QTR calibration cycles at startup
const int calibrationCycles = 400;

// Target center for readLine() (range 0..(1000*(NUM_SENSORS-1)))
const int targetPosition = 1007; // approximate center (for 8 sensors range 0..7000)

// ========== SETUP ==========
void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(leftIn1Pin, OUTPUT);
  pinMode(leftIn2Pin, OUTPUT);
  pinMode(rightIn1Pin, OUTPUT);
  pinMode(rightIn2Pin, OUTPUT);

  // QTR init
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  //qtr.setEmitterPin(9); // optional: LED emitter pin (set to 255 or -1 if not used)

  Serial.println("QTR-8 PID Line Follower");
  Serial.println("Calibrating - move robot over line slowly for ~4 seconds");

  // Calibrate: rotate/slide robot over line while calibration runs
  for (int i = 0; i < calibrationCycles; i++) {
    qtr.calibrate(); // collects min/max for each sensor
    delay(5);
  }
  Serial.println("Calibration done.");
  Serial.println("Starting main loop...");

  lastTime = millis();
}

// ========== MAIN LOOP ==========
void loop() {
  unsigned long now = millis();
  if (now - lastTime >= sampleTime) {
    unsigned long dt_ms = now - lastTime;
    lastTime = now;

    // Read position: returns 0..1000*(NUM_SENSORS-1) — for 8 sensors -> 0..7000
      uint16_t position = qtr.readLineBlack(sensorValues);

    // Compute error (positive if robot is to the right of target)
    double error = (double)position - (double)targetPosition;

    // PID calculations (discrete)
    integral += error * (dt_ms / 1000.0);
    // clamp integral to avoid windup
    if (integral > integralMax) integral = integralMax;
    if (integral < -integralMax) integral = -integralMax;

    double derivative = 0;
    if (dt_ms > 0) derivative = (error - lastError) / (dt_ms / 1000.0);

    double output = Kp * error /*+ Ki * integral + Kd * derivative*/;

    lastError = error;

    // Apply correction: 'output' is a signed turn command
    // Convert to motor PWM adjustments
    // left = base + turn, right = base - turn
    double leftSpeed_d  = baseSpeed + output;
    double rightSpeed_d = baseSpeed - output;

    // Constrain speeds
    int leftPWM  = constrain((int)round(leftSpeed_d), PWM_MIN, PWM_MAX);
    int rightPWM = constrain((int)round(rightSpeed_d), PWM_MIN, PWM_MAX);

    // Drive motors
    driveMotor(leftPWM,  rightPWM);

    // Debugging
    Serial.print("pos:");
    Serial.print(position);
    Serial.print(" err:");
    Serial.print(error, 2);
    Serial.print(" out:");
    Serial.print(output, 2);
    Serial.print(" Lpwm:");
    Serial.print(leftPWM);
    Serial.print(" Rpwm:");
    Serial.println(rightPWM);
  }

  // small yield so serial / USB can handle things
  delay(1);
}

// ========== MOTOR FUNCTION ==========
// This assumes direction pins: IN1 HIGH + IN2 LOW -> forward
//                             IN1 LOW  + IN2 HIGH -> reverse
void driveMotor(int leftPWM, int rightPWM) {
  // LEFT motor
  if (leftPWM >= 0) {
    analogWrite(leftIn1Pin, leftPWM);  // HIGH = forward, change if reversed
    analogWrite(leftIn2Pin, 0);    // Speed (0–255)
  }

  // RIGHT motor
  if (rightPWM >= 0) {
    analogWrite(rightIn1Pin, rightPWM);  // HIGH = forward, change if reversed
    analogWrite(rightIn2Pin, 0);  
  } 
}
