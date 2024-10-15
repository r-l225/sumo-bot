int pinIR = 2; 

// Ultrasonic Sensor Pins
const int TrigPin = 11; // Trigger pin for ultrasonic sensor
const int EchoPin = 6;  // Echo pin for ultrasonic sensor

// Time limits and thresholds
const int detectionThreshold = 20;  // Distance to charge at the opponent (in cm)
const int backupTime = 1000;        // Time to reverse after detecting boundary (in ms)
const unsigned long searchTimeout = 5000;  // Timeout for SEARCHING (in ms)

enum State {
  SEARCHING,       // Searching for opponent
  MOVING_FORWARD,  // Moving toward opponent
  BACKING_UP,      // Reversing after boundary detection
  STOPPED          // Stopped state
};

State currentState = SEARCHING; // Initial state

unsigned long previousMillis = 0;    // Store time for BACKING_UP and SEARCHING state
unsigned long searchStartTime = 0;   // Store the start time of the SEARCHING state

void setup() {
  Serial.begin(9600);
  pinMode(pinIR, INPUT);

  // Ultrasonic Sensor Pins Setup
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  Serial.println("Testing state machine with motor control placeholders.");
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

// Placeholder for moving the robot forward
void forwardMotors() {
  Serial.println("Motors moving forward...");
  // Add actual motor control logic here
}

// Placeholder for rotating the robot in search mode
void rotateMotors() {
  Serial.println("Motors rotating to search...");
  // Add actual motor control logic here
}

// Placeholder for stopping the motors
void stopMotors() {
  Serial.println("Motors stopped.");
  // Add actual motor control logic here
}

// Placeholder for reversing the robot
void reverseMotors() {
  Serial.println("Motors reversing...");
  // Add actual motor control logic here
}

void loop() {
  int IRstate = digitalRead(pinIR);  // Read the IR sensor state
  long distance = getDistance();     // Get the distance from the ultrasonic sensor

  Serial.print("Distance: ");
  Serial.println(distance);          // Print the distance
  Serial.print("IR Sensor State: ");
  Serial.println(IRstate);           // Print the IR sensor state

  // State machine logic
  switch (currentState) {
    case SEARCHING:
      Serial.println("Current State: SEARCHING");
      rotateMotors();  // Rotate to search for opponent

      if (distance < detectionThreshold) {
        currentState = MOVING_FORWARD;
        stopMotors();  // Stop rotating
        Serial.println("Transition to MOVING_FORWARD: Opponent detected.");
      } else {
        // Start rotating if searching, and track how long we've been searching
        if (searchStartTime == 0) {
          searchStartTime = millis();  // Start the search timer
          Serial.println("Starting search timer.");
        }

        // Check if the search timeout has been exceeded
        unsigned long currentTime = millis();
        Serial.print("Time spent searching: ");
        Serial.println(currentTime - searchStartTime);

        if (currentTime - searchStartTime >= searchTimeout) {
          Serial.println("Search timeout exceeded. Transition to MOVING_FORWARD.");
          currentState = MOVING_FORWARD;  // Stop searching and just move forward
          searchStartTime = 0;            // Reset the search timer
        }
      }
      break;

    case MOVING_FORWARD:
      Serial.println("Current State: MOVING_FORWARD");
      forwardMotors();  // Move toward opponent

      if (IRstate == LOW) {
        currentState = BACKING_UP;  // Boundary detected, start backing up
        previousMillis = millis();  // Record the time for backing up
        stopMotors();  // Stop forward motion
        Serial.println("Transition to BACKING_UP: Boundary detected.");
      } else if (distance >= detectionThreshold) {
        currentState = SEARCHING;  // Lost sight of opponent, go back to searching
        searchStartTime = millis();  // Reset the search timer when switching back to SEARCHING
        Serial.println("Opponent lost, transition to SEARCHING.");
      }
      break;

    case BACKING_UP:
      Serial.println("Current State: BACKING_UP");
      reverseMotors();  // Reverse to avoid boundary

      if (millis() - previousMillis >= backupTime) {
        currentState = SEARCHING;  // After backing up, start searching again
        searchStartTime = millis();  // Reset the search timer when switching back to SEARCHING
        Serial.println("Finished backing up, returning to SEARCHING.");
      }
      break;

    case STOPPED:
      Serial.println("Current State: STOPPED");
      stopMotors();  // Stop the robot
      break;
  }

  delay(500);  // Small delay to smooth out sensor readings and actions
}

