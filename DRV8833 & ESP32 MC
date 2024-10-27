// Motor control pins
const int L_IN1 = 2;   // Control pin 1 for left motor
const int L_IN2 = 0;   // Control pin 2 for left motor
const int R_IN3 = 4;   // Control pin 1 for right motor
const int R_IN4 = 16;  // Control pin 2 for right motor

// PWM frequency and resolution settings for ESP32
const int PWMFreq = 1000;       // PWM frequency in Hz
const int PWMResolution = 8;    // PWM resolution (8 bits)
const int MAX_SPEED = 255;      // Maximum PWM duty cycle
const int MIN_SPEED = 0;        // Minimum PWM duty cycle
const int STEP_DELAY = 1000;      // Delay between speed steps (in milliseconds)

void setup() {
  // Set up for PWM channels and pins
  ledcAttach(L_IN1,  PWMFreq, PWMResolution);
  ledcAttach(L_IN2,  PWMFreq, PWMResolution);
  ledcAttach(R_IN3,  PWMFreq, PWMResolution);
  ledcAttach(R_IN4,  PWMFreq, PWMResolution);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Robot searches 
  RotateInPlace();
  delay(10000);
}

// Function to stop both motors
void StopMotors() {
  // Set all motor pins to zero for full stop
  ledcWrite(L_IN1, 0);
  ledcWrite(L_IN2, 0);
  ledcWrite(R_IN3, 0);
  ledcWrite(R_IN4, 0);
}

void RotateInPlace() {
  // Rotate clockwise by driving left motor forward and right motor backward
  ledcWrite(L_IN1, 200);  // Medium speed forward for left motor
  ledcWrite(L_IN2, 0);    // Left motor IN2 off

  ledcWrite(R_IN3, 0);    // Right motor IN3 off
  ledcWrite(R_IN4, 200);  // Medium speed backward for right motor
}

// Robot moves backward
void Reverse() {
  // Move both motors backward
  ledcWrite(L_IN1, 0);    // Left motor IN1 off
  ledcWrite(L_IN2, 200);  // Medium speed backward for left motor

  ledcWrite(R_IN3, 0);    // Right motor IN3 off
  ledcWrite(R_IN4, 200);  // Medium speed backward for right motor
}

// Robot moves forward
void Forward() {
  // Move both motors forward
  ledcWrite(L_IN1, 255);  // Full speed forward for left motor
  ledcWrite(L_IN2, 0);    // Left motor IN2 off

  ledcWrite(R_IN3, 255);  // Full speed forward for right motor
  ledcWrite(R_IN4, 0);    // Right motor IN4 off

}
// Function to gradually increase speed for forward movement
void accelerateForward(int targetSpeed) {
  for (int speed = MIN_SPEED; speed <= targetSpeed; speed += 10) {
    ledcWrite(L_IN1, speed);  // Left motor IN1 forward
    ledcWrite(L_IN2, 0);      // Left motor IN2 off
    ledcWrite(R_IN3, speed);  // Right motor IN3 forward
    ledcWrite(R_IN4, 0);      // Right motor IN4 off
    delay(STEP_DELAY);        // Delay between speed steps
  }
  Serial.println("Accelerating Forward");
}

// Function to gradually increase speed for backward movement
void accelerateBackward(int targetSpeed) {
  for (int speed = MIN_SPEED; speed <= targetSpeed; speed += 10) {
    ledcWrite(L_IN1, 0);      // Left motor IN1 off
    ledcWrite(L_IN2, speed);  // Left motor IN2 backward
    ledcWrite(R_IN3, 0);      // Right motor IN3 off
    ledcWrite(R_IN4, speed);  // Right motor IN4 backward
    delay(STEP_DELAY);        // Delay between speed steps
  }
  Serial.println("Accelerating Backward");
}

// Function to gradually decrease speed to stop
void decelerateStop() {
  int currentSpeed = MAX_SPEED;
  while (currentSpeed > MIN_SPEED) {
    ledcWrite(L_IN1, currentSpeed);  // Left motor IN1
    ledcWrite(L_IN2, 0);             // Left motor IN2 off
    ledcWrite(R_IN3, currentSpeed);  // Right motor IN3
    ledcWrite(R_IN4, 0);             // Right motor IN4 off
    currentSpeed -= 10;              // Reduce speed gradually
    delay(STEP_DELAY);               // Delay between speed steps
  }
  StopMotors();
  Serial.println("Decelerating to Stop");
}


