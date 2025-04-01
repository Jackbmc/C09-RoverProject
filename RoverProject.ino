// === Motor Pins ===
const int LEFT_EN = 3;
const int LEFT_IN1 = 8;
const int LEFT_IN2 = 9;

const int RIGHT_EN = 4;
const int RIGHT_IN1 = 10;
const int RIGHT_IN2 = 11;

// === Ultrasonic Sensor Pins ===
const int trigPin = 7;
const int echoPin = 12;

void setup() {
  // Motor setup
  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // Sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Enable motors at full speed
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, HIGH);

  Serial.begin(9600); // Optional for debugging
}

void loop() {
  long distance = getDistanceCM();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 10) {
    // Move forward
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
  } else {
    // Stop
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
  }

  delay(100); // check every 100ms
}

// Function to get distance in cm
long getDistanceCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // Timeout after 20ms
  long cm = duration * 0.034 / 2;

  // Cap max distance (if no object is detected)
  if (cm == 0 || cm > 400) return 400;

  return cm;
}
