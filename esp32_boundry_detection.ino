int pinIR = 5; 
const int TrigPin = 25;
const int EchoPin = 26;  

// Time limits and thresholds
const int detectionThreshold = 20;  // Distance to charge at the opponent (in cm)
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
  ROTATING, 
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
    if (IRstate == LOW) {
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
    switch (currentState) {
      case SEARCHING:
        Serial.println("Current State: SEARCHING");
        rotateMotors(); 

        if (opponentDetected) {
          currentState = MOVING_FORWARD;
          stopMotors(); 
          Serial.println("Transition to MOVING_FORWARD: Opponent detected.");
        } else {
          // Check if the search timeout has been exceeded
          unsigned long currentTime = millis();
          if (searchStartTime == 0) {
            searchStartTime = currentTime;  // Start the search timer
          }
          
          if (currentTime - searchStartTime >= searchTimeout) {
            Serial.println("Search timeout exceeded. Transition to MOVING_FORWARD.");
            currentState = MOVING_FORWARD;
            searchStartTime = 0;  // Reset search timer
          }
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
          searchStartTime = millis();  // Reset the search timer
          Serial.println("Opponent lost, transition to SEARCHING.");
        }
        break;

      case BACKING_UP:
        Serial.println("Current State: BACKING_UP");
        reverseMotors();  

        if (millis() - previousMillis >= backupTime) {
          currentState = ROTATING; 
          previousMillis = millis();  // Reset timer for rotation
          stopMotors();  // Stop reversing
          Serial.println("Finished backing up, now rotating.");
        }
        break;

      case ROTATING:
        Serial.println("Current State: ROTATING");
        rotateMotors();  // Rotate after backing up

        if (millis() - previousMillis >= rotateTime) {
          currentState = MOVING_FORWARD;  // After rotating, move forward again
          previousMillis = millis();  // Reset timer for forward motion
          stopMotors();  // Stop rotating
          Serial.println("Finished rotating, moving forward again.");
        }
        break;

      case STOPPED:
        Serial.println("Current State: STOPPED");
        stopMotors();  // Stop the robot
        break;
    }

    // Small delay to avoid flooding the system with actions
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Motor control placeholder functions
void forwardMotors() {
  Serial.println("Motors moving forward...");
  // Add actual motor control logic here
}

void rotateMotors() {
  Serial.println("Motors rotating...");
}

void stopMotors() {
  Serial.println("Motors stopped.");
}

void reverseMotors() {
  Serial.println("Motors reversing...");
}

