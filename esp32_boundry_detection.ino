int pinIR = 5; 
const int TrigPin = 25;
const int EchoPin = 26;  

// Motor Control Pins
const int L_PWM = 15; // PWM control for left motor
const int L_IN1 = 32; // Direction control for left motor
const int L_IN2 = 14; // Direction control for left motor
const int R_PWM = 33; // PWM control for right motor
const int R_IN1 = 13; // Direction control for right motor
const int R_IN2 = 12; // Direction control for right motor

// Time limits and thresholds
const int detectionThreshold = 80;  // Distance to charge at the opponent (in cm)
const int backupTime = 2000;        // Time to reverse after detecting boundary (in ms)
const int rotateTime = 3000;  
const unsigned long searchTimeout = 5000;  // Timeout for SEARCHING 

// Shared flags
volatile bool boundaryDetected = false;
volatile bool opponentDetected = false;

// State machine variables
enum State {
  SEARCHING,       // Searching for opponent
  MOVING_FORWARD,  // Moving toward opponent
  BACKING_UP,      // Reversing after boundary detection 
  STOPPED          // Stopped state
};

State currentState = SEARCHING; // Initial state

unsigned long previousMillis = 0;    // Store time for BACKING_UP and SEARCHING state
unsigned long searchStartTime = 0;   // Store the start time of the SEARCHING state

void TaskSensors(void *pvParameters);
void TaskNavigation(void *pvParameters);
long getDistance();
void forwardMotors();
void rotateMotors();
void stopMotors();
void reverseMotors();

void setup() {
  Serial.begin(115200);

  pinMode(pinIR, INPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  pinMode(pinIR, INPUT);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  xTaskCreatePinnedToCore(
    TaskSensors,    // Task for sensor data collection (core 1)
    "TaskSensors",  // Name of the task
    10000,          // Stack size
    NULL,           // Task input parameter
    1,              // Priority
    NULL,           // Task handle
    0);             // Core to run task on (Core 0)

  xTaskCreatePinnedToCore(
    TaskNavigation, // Task for navigation and motor control (core 2)
    "TaskNavigation", 
    10000, 
    NULL, 
    1, 
    NULL, 
    1);             // Core to run task on (Core 1)
}

void loop() {
}

long getDistance() {
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  long duration = pulseIn(EchoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

// Task for Core 1: Sensor data collection
void TaskSensors(void *pvParameters) {
  for (;;) {
    // Read the IR sensor 
    int IRstate = digitalRead(pinIR);
    
    // Get distance from ultrasonic sensor
    long distance = getDistance();

    // Update flags based on sensor readings
    if (IRstate == HIGH) {
      boundaryDetected = true;  // Boundary detected
    } else {
      boundaryDetected = false;  // No boundary
    }

    if (distance < detectionThreshold) {
      opponentDetected = true;  // Opponent detected
    } else {
      opponentDetected = false;  // No opponent detected
    }

    // Short delay to avoid flooding the system with too frequent sensor readings
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Task for Core 2: Navigation and motor control
void TaskNavigation(void *pvParameters) {
  for (;;) {

    // Check boundary detection first (IR sensor has more priority)
    if (boundaryDetected) {
      currentState = BACKING_UP;
      previousMillis = millis();  // Record the time for backing up
      stopMotors();  // Stop any ongoing motion
      Serial.println("Boundary detected! Transition to BACKING_UP.");
    }

    switch (currentState) {
      case SEARCHING:
        Serial.println("Current State: SEARCHING");
        rotateMotors();  // Rotate to search for opponent

        if (opponentDetected) {
          currentState = MOVING_FORWARD;
          stopMotors();  // Stop rotating
          Serial.println("Transition to MOVING_FORWARD: Opponent detected.");
        }
        break;

      case MOVING_FORWARD:
        Serial.println("Current State: MOVING_FORWARD");
        forwardMotors();  // Move toward opponent

        if (boundaryDetected) {
          currentState = BACKING_UP;
          previousMillis = millis();  // Record the time for backing up
          stopMotors();  // Stop forward motion
          Serial.println("Transition to BACKING_UP: Boundary detected.");
        } else if (!opponentDetected) {
          currentState = SEARCHING;  // Opponent lost, go back to searching
          Serial.println("Opponent lost, transition to SEARCHING.");
        }
        break;

      case BACKING_UP:
        Serial.println("Current State: BACKING_UP");
        reverseMotors();  // Reverse to avoid boundary

        if (millis() - previousMillis >= backupTime) {
          currentState = SEARCHING;  // After backing up, start SEARCHING
          previousMillis = millis();  // Reset timer for rotation
          stopMotors();  // Stop reversing
          Serial.println("Finished backing up, now searching.");
        }
        break;

      case STOPPED:
        Serial.println("Current State: STOPPED");
        stopMotors();  // Stop the robot
        break;
    }

    // Small delay to avoid flooding the system with actions
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Motor control placholder functions
void forwardMotors() {

  Serial.println("Motors reversing...");
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, 255);  // Full speed backward for left motor

  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
  analogWrite(R_PWM, 255);
}

void rotateMotors() {
  Serial.println("Motors rotating...");
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, 150);  // Medium speed forward for left motor

  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
  analogWrite(R_PWM, 150); 
}

void stopMotors() {
  Serial.println("Motors stopped.");
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, 0);  // Stop PWM for left motor

  // Stop right motor
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  analogWrite(R_PWM, 0); 
}

void reverseMotors() {
   Serial.println("Motors moving BACKWARDS...");
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  analogWrite(L_PWM, 255);  // Full speed forward for left motor

  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
  analogWrite(R_PWM, 255); 
}



