int pinIR = 2; 

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

// Time limits and thresholds
const int detectionThreshold = 20; // Distance to charge at the opponent (in cm)
const int backupTime = 1000;       // Time to reverse after detecting boundary (in ms)
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
  pinMode(L_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  
  // Ultrasonic Sensor Pins Setup
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  Serial.println("Starting Robo-Car with state machine.");
}

void moveForward() {
  Serial.println("Moving Forward.");
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, 255);  // Full speed forward for left motor

  digitalWrite(R_IN1, HIGH);
  digitalWrite(R_IN2, LOW);
  analogWrite(R_PWM, 255);  // Full speed forward for right motor
}

void stopMoving() {
  Serial.println("Stopped.");
  analogWrite(L_PWM, 0);  // Stop left motor
  analogWrite(R_PWM, 0);  // Stop right motor
}

void moveBackward() {
  Serial.println("Moving Backward.");
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  analogWrite(L_PWM, 255);  // Full speed backward for left motor

  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
  analogWrite(R_PWM, 255);  // Full speed backward for right motor
}

void rotateInPlace() {
  Serial.println("Rotating.");
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, 200);  // Medium speed forward for left motor

  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
  analogWrite(R_PWM, 200);  // Medium speed backward for right motor
}

// Function to calculate distance using ultrasonic sensor
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

void loop() {
  int IRstate = digitalRead(pinIR);
  long distance = getDistance();

  Serial.print("Distance: ");
  Serial.println(distance);

  switch (currentState) {
    case SEARCHING:
      if (distance < detectionThreshold) {
        currentState = MOVING_FORWARD;
        searchStartTime = 0;  // Reset the search timer
        Serial.println("Opponent detected, moving forward.");
      } else {
        // Start rotating if searching, and track how long we’ve been searching
        rotateInPlace();
        if (searchStartTime == 0) {
          searchStartTime = millis();  // Start the search timer
          Serial.println("Starting search timer.");
        }

        // Check if the search timeout has been exceeded
        unsigned long currentTime = millis();
        Serial.print("Time spent searching: ");
        Serial.println(currentTime - searchStartTime);
        
        if (currentTime - searchStartTime >= searchTimeout) {
          Serial.println("Search timeout exceeded. Moving forward.");
          currentState = MOVING_FORWARD;  // Stop searching and just move forward
          searchStartTime = 0;  // Reset the search timer
        }
      }
      break;

    case MOVING_FORWARD:
      if (IRstate == HIGH) {
        currentState = BACKING_UP;  // Boundary detected, start backing up
        previousMillis = millis();  // Record the time for backing up
        Serial.println("Boundary detected, backing up.");
      } else if (distance >= detectionThreshold) {
        currentState = SEARCHING;  // Lost sight of opponent, go back to searching
        searchStartTime = millis();  // Reset the search timer when switching back to SEARCHING
        Serial.println("Opponent lost, searching again.");
      } else {
        moveForward();  // Continue moving toward the opponent
      }
      break;

    case BACKING_UP:
      moveBackward();
      Serial.print("Time spent backing up: ");
      Serial.println(millis() - previousMillis);
      if (millis() - previousMillis >= backupTime) {
        currentState = SEARCHING;  // After backing up, start searching again
        searchStartTime = millis();  // Reset the search timer when switching back to SEARCHING
        Serial.println("Finished backing up, returning to searching.");
      }
      break;

    case STOPPED:
      stopMoving();  // Stop the bot for any reason
      break;
  }

  delay(400);  // Small delay to smooth out sensor readings and actions
}
