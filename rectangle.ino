// Motor Pins
const int LEFT_EN = 3;
const int LEFT_IN1 = 8;
const int LEFT_IN2 = 9;

const int RIGHT_EN = 4;
const int RIGHT_IN1 = 10;
const int RIGHT_IN2 = 11;

// HC-SR04 Pins
const int trigPin = 7;
const int echoPin = 12;

// Distance in cm
const int SHORT_SIDE_CM = 25;
const int LONG_SIDE_CM = 40;

// Time to travel 1 cm
const float TIME_PER_CM = 50;

// Time to make 90º turn
const int TURN_TIME = 2050; // milliseconds

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

  // Enable motors
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, HIGH);

  Serial.begin(9600);
}

void loop() {
  // Rectangle loop
  moveForward(LONG_SIDE_CM);
  turnRight();

  moveForward(SHORT_SIDE_CM);
  turnRight();

  moveForward(LONG_SIDE_CM);
  turnRight();

  moveForward(SHORT_SIDE_CM);
  turnRight();
}

// Movement Helpers

void moveForward(int distanceCM) {
  unsigned long moveTime = distanceCM * TIME_PER_CM;

  // Start motors forward
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);

  delay(moveTime); // Move for calculated time

  // Stop motors
  stopMotors();
}

void turnRight() {
  // Turn right in place (left motor forward, right motor backward)
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);

  delay(TURN_TIME);

  stopMotors();
}

void stopMotors() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);

  delay(300);
}
