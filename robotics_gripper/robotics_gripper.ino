#include <Servo.h>

Servo gripperServo;
Servo verticalServo;

int gripperPos;
int verticalPos;

const int gripperPin = 2;
const int verticalPin = 3;

const int trigPin = 3;
const int echoPin = 3;

const int DISTANCE_FROM_GRIPPER = 60;
const int GRIPPER_CLOSED_POS = 10;
const int GRIPPER_OPEN_POS = 55;
const int GRIPPER_DROPOFF_POS = 15;

const int VERTICAL_DOWN_POS = 0;
const int GRIPPER_UP_POS = 100;

void setup() {
  gripperServo.attatch(gripperPin);
  verticalServo.attach(verticalPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  gripperServo.write(GRIPPER_OPEN_POS);
  verticalServo.write(VERTICAL_DOWN_POS);
}

void loop() {
  long duration, distance;
  // Read distance
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = microsecondsToCentimeters(duration);

  Serial.print(distance);
  Serial.println("cm");

  if(distance <= DISTANCE_FROM_GRIPPER){ // Might be inacurate when close range in practice so might have to do it "blind", i.e move forward x amount, then do closing
    pickUpAndStore();
  }
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