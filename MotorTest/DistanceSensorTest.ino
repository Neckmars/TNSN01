/*#define TRIG_PIN 9
#define ECHO_PIN 10

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  long duration = 0;
  // Send a 10µs pulse to trigger the sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the time it takes for the echo to return
  duration = pulseIn(ECHO_PIN, HIGH);

  // Convert to distance (in cm)
  long distance = duration * 0.34 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" mm");

  delay(200);
}*/
