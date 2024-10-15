// Motor Control Pins
const int L_PWM = 9; // PWM control for left motor
const int L_IN1 = 2; // Direction control for left motor
const int L_IN2 = 3; // Direction control for left motor
const int R_PWM = 10; // PWM control for right motor
const int R_IN1 = 4; // Direction control for right motor
const int R_IN2 = 5; // Direction control for right motor

// Ultrasonic Sensor Pins
const int TrigPin = 11; // Trigger pin for ultrasonic sensor
const int EchoPin = 6; // Echo pin for ultrasonic sensor

// Distance threshold for detecting another robot (in cm)
const int detectionThreshold = 10; // Distance to charge at the opponent

void setup() {
  // Motor Pins Setup
  pinMode(L_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  
  // Ultrasonic Sensor Pins Setup
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  
  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Get distance from ultrasonic sensor
  long distance = getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  // Decision Making based on distance
  if (distance < detectionThreshold) {
    // Opponent detected: Charge 
    chargeAtOpponent();
  } else {
    // No opponent detected: Rotate in place 
    rotateInPlace();
  }
  
  delay(100); // Delay between sensor readings
}

// Function to charge forward towards opponent
void chargeAtOpponent() {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, 255);  // Full speed forward for left motor

  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
  analogWrite(R_PWM, 255);  // Full speed forward for right motor
}

// Function to rotate in place (looking for opponent)
void rotateInPlace() {
  // Rotate clockwise by driving left motor forward and right motor backward
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, 200);  // Medium speed forward for left motor

  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
  analogWrite(R_PWM, 200);  // Medium speed backward for right motor
}

// Function to calculate distance using ultrasonic sensor
long getDistance() {
  // Send a 10 microsecond pulse to the trigger pin
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  // Read the time it takes for the echo to return
  long duration = pulseIn(EchoPin, HIGH);

  // Convert time to distance (cm)
  long distance = duration * 0.034 / 2;
  return distance;
}
