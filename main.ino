#include <Wire.h>
#include <Servo.h>

// Pin definitions
const int enableMotor1 = 3;  // PWM pin for Left Motor
const int in1 = 8;           // Left Motor direction
const int in2 = 9;           // Left Motor direction
const int enableMotor2 = 4;  // PWM pin for Right Motor
const int in3 = 10;          // Right Motor direction
const int in4 = 11;          // Right Motor direction

const int trigPin = 7;
const int echoPin = 12;
const int servoPin = 6;

// Servo angles for scanning
const int SERVO_LEFT_ANGLE = 135;   // Rover's left
const int SERVO_CENTER_ANGLE = 90;
const int SERVO_RIGHT_ANGLE = 45;   // Rover's right

// Distances and Speed
const float WALL_DETECT_DISTANCE = 10.0;     // cm, objects closer than this are walls
const float WALL_THRESHOLD_OPEN = 20.0;     // cm, if opening is larger than this, it's considered a path
const int MOTOR_BASE_SPEED = 130;          // Base speed for motors (0-255)
const int MOTOR_TURN_SPEED = 100;          // Speed for turning
const unsigned long MOVE_FORWARD_DURATION_MS = 1200; // Duration for one "unit" of forward movement

// MPU6050 Gyro Variables
const int MPU_ADDR = 0x68;
int16_t GyZ_raw;                 // Raw Z-axis gyro data
float gyroZBias = 0.0;           // Gyro Z-axis bias offset
float currentAngleZ = 0.0;       // Rover's current angle in degrees
float targetAngleZ = 0.0;        // Rover's target angle for straight movement
unsigned long lastTimeIntegration = 0;
const float GYRO_SENSITIVITY = 131.0; // LSB/(degrees/s) for MPU6050 default settings (FS_SEL=0)
const int GYRO_CALIBRATION_SAMPLES = 1000;

// PID for Straight Movement
const float KP_STRAIGHT = 2.5;   // Proportional gain for heading correction

// Turn Control
const float TURN_ANGLE_TOLERANCE = 3.0; // degrees, how close to target angle for turn to be "complete"

Servo myServo;

void setup() {
  Serial.begin(9600);
  Serial.println("Maze Solving Rover Initializing...");

  // Motor Pin Setup
  pinMode(enableMotor1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enableMotor2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  stopMotors(); // Ensure motors are off

  // Ultrasonic Sensor Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Servo Setup
  myServo.attach(servoPin);
  myServo.write(SERVO_CENTER_ANGLE); // Start centered
  delay(500); // Allow servo to reach position

  // MPU6050 Setup
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("MPU6050 Initialized.");

  calibrateGyro(); // Calibrate Gyro Z-axis

  lastTimeIntegration = micros(); // Initialize time for angle integration
  Serial.println("Setup Complete. Starting Maze Solving.");
  delay(1000);
}

void loop() {
  updateCurrentAngleZ(); // Continuously update the rover's angle
  mazeLogic();           // Execute the maze-solving decision logic
  delay(50);             // Short delay in the main loop
}

void calibrateGyro() {
  Serial.println("Calibrating Gyro... Keep rover stationary!");
  long gyZTotal = 0;
  
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); // Starting register for GyZ (GYRO_ZOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true); // Read 2 bytes (GyZ H and L)
    int16_t rawZ = Wire.read() << 8 | Wire.read();
    gyZTotal += rawZ;
    delay(3); // Small delay between readings
  }
  gyroZBias = (float)gyZTotal / GYRO_CALIBRATION_SAMPLES;
  Serial.print("Gyro Z Bias calculated: ");
  Serial.println(gyroZBias);
}

void updateCurrentAngleZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true); // Read GyZ H and L
  GyZ_raw = Wire.read() << 8 | Wire.read();

  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastTimeIntegration) / 1000000.0; // Convert to seconds
  lastTimeIntegration = currentTime;

  float angularVelocityZ = (GyZ_raw - gyroZBias) / GYRO_SENSITIVITY;

  // Integrate to get angle
  currentAngleZ += angularVelocityZ * deltaTime;
}

float getDistanceCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH, 25000); // Added timeout (approx 4m range)
  float distance = (duration * 0.0343) / 2.0;
  if (distance == 0 || distance > 400) { // If timeout or out of sensible range
    return 400.0; // Return a large distance
  }
  return distance;
}

// Scans Right, Front, Left and stores distances
void scanEnvironment(float &distR, float &distF, float &distL) {
  Serial.println("Scanning...");
  myServo.write(SERVO_RIGHT_ANGLE);
  delay(400); // Allow servo to move and sensor to stabilize
  distR = getDistanceCm();
  Serial.print("  Dist Right: "); Serial.println(distR);

  myServo.write(SERVO_CENTER_ANGLE);
  delay(400);
  distF = getDistanceCm();
  Serial.print("  Dist Front: "); Serial.println(distF);

  myServo.write(SERVO_LEFT_ANGLE);
  delay(400);
  distL = getDistanceCm();
  Serial.print("  Dist Left: "); Serial.println(distL);

  myServo.write(SERVO_CENTER_ANGLE); // Return to center
  delay(300);
}

// --- Motor Control Functions ---
void setMotorSpeeds(int leftSpeed, int rightSpeed, bool leftForward, bool rightForward) {
  // Left Motor
  digitalWrite(in1, leftForward ? HIGH : LOW);
  digitalWrite(in2, leftForward ? LOW : HIGH);
  analogWrite(enableMotor1, constrain(abs(leftSpeed), 0, 255));

  // Right Motor
  digitalWrite(in3, rightForward ? HIGH : LOW);
  digitalWrite(in4, rightForward ? LOW : HIGH);
  analogWrite(enableMotor2, constrain(abs(rightSpeed), 0, 255));
}

void stopMotors() {
  setMotorSpeeds(0, 0, true, true); // Speed 0, direction doesn't matter
  Serial.println("Motors Stopped");
}

void moveForwardPID() {
  Serial.print("Moving Forward. Target Angle: "); Serial.println(targetAngleZ);
  unsigned long startTime = millis();

  while (millis() - startTime < MOVE_FORWARD_DURATION_MS) {
    updateCurrentAngleZ();
    float error = targetAngleZ - currentAngleZ;

    // Normalize error for heading -180 to 180
    while (error > 180.0) error -= 360.0;
    while (error < -180.0) error += 360.0;
    
    float correction = KP_STRAIGHT * error;

    int leftSpeed = MOTOR_BASE_SPEED - (int)correction;
    int rightSpeed = MOTOR_BASE_SPEED + (int)correction;
    
    setMotorSpeeds(leftSpeed, rightSpeed, true, true); // Both forward
    delay(20); // Control loop delay
  }
  stopMotors();
  Serial.println("Forward Movement Complete.");
}

void performTurn(float relativeTurnAngle) {
  Serial.print("Performing Turn: "); Serial.print(relativeTurnAngle);
  Serial.print(" deg. From: "); Serial.print(currentAngleZ);
  
  float turnInitialAngle = currentAngleZ;
  targetAngleZ += relativeTurnAngle; // Update the global target angle

  Serial.print(" To Target: "); Serial.println(targetAngleZ);

  if (relativeTurnAngle > 0) { // Turn Right (Clockwise)
    // Left motor forward, Right motor backward
    setMotorSpeeds(MOTOR_TURN_SPEED, MOTOR_TURN_SPEED, true, false);
  } else { // Turn Left (Counter-Clockwise)
    // Left motor backward, Right motor forward
    setMotorSpeeds(MOTOR_TURN_SPEED, MOTOR_TURN_SPEED, false, true);
  }

  // Loop until the turn is approximately complete
  // This measures total rotation from start of turn.
  while (true) {
    updateCurrentAngleZ();
    float angleTurned = currentAngleZ - turnInitialAngle;

    // Normalize angleTurned if currentAngleZ wraps around 0/360
    // This simple check assumes turns are less than 180 at a time mostly
    if (abs(relativeTurnAngle - angleTurned) > 180 && relativeTurnAngle != 0) {
         if (relativeTurnAngle > 0 && angleTurned < 0) angleTurned += 360; // e.g. target 10, current -350, turned 10
         else if (relativeTurnAngle < 0 && angleTurned > 0) angleTurned -= 360; // e.g. target -10, current 350, turned -10
    }

    if (abs(angleTurned) >= abs(relativeTurnAngle) - TURN_ANGLE_TOLERANCE) {
      break; 
    }
    delay(10); // Loop delay for turn
  }
  
  stopMotors();
  currentAngleZ = targetAngleZ; // Aggressively set current angle to target to correct minor overshoot/undershoot
  Serial.print("Turn Complete. New Angle: "); Serial.println(currentAngleZ);
  delay(200); // Pause after turn
}


// --- Maze Solving Logic ---
void mazeLogic() {
    float distR, distF, distL;
    scanEnvironment(distR, distF, distL);

    Serial.print("Scan Results -> R: "); Serial.print(distR);
    Serial.print(" cm, F: "); Serial.print(distF);
    Serial.print(" cm, L: "); Serial.println(distL);

    if (distR > WALL_THRESHOLD_OPEN) {
        Serial.println("Decision: Path open to Right. Turning Right.");
        performTurn(90.0);      // Positive for right turn
        moveForwardPID();
    } else if (distF > WALL_THRESHOLD_OPEN) {
        Serial.println("Decision: Path open Forward. Moving Forward.");
        moveForwardPID();
    } else if (distL > WALL_THRESHOLD_OPEN) {
        Serial.println("Decision: Path open to Left. Turning Left.");
        performTurn(-90.0);     // Negative for left turn
        moveForwardPID();
    } else {
        Serial.println("Decision: Dead end. Turning Around (180 deg).");
        performTurn(180.0);    // Turn around
    }
    delay(100); // Short delay after an action sequence
}
