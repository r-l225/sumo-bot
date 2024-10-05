#define IRpinLeft 2  
#define IRpinRight 3 

#define motorLeftForward 4
#define motorRightForward 5


bool dummyLeftIR = HIGH;   
bool dummyRightIR = HIGH;

void setup() {

  pinMode(IRpinLeft, INPUT);
  pinMode(IRpinRight, INPUT);
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorRightForward, OUTPUT);

  // Start with motors off
  stopMoving();

  Serial.begin(9600);
}

int readDummyLeftIR() {
  return dummyLeftIR; 
}

int readDummyRightIR() {
  return dummyRightIR; 
}

void moveForward() {
  Serial.println("Moving Forward: Both wheels are moving.");
}

void stopMoving() {
  Serial.println("Stopped: Both wheels are stopped.");
}

void turnLeft() {
  Serial.println("Turning Left: Left wheel stopped, right wheel moving.");
}

void turnRight() {
  Serial.println("Turning Right: Right wheel stopped, left wheel moving.");
}

void loop() {
  // Read dummy IR sensor values 
  int leftIR = readDummyLeftIR();   
  int rightIR = readDummyRightIR(); 

  moveForward();
  
  // If the left IR sensor detects a border 
  if (leftIR == LOW) {
    stopMoving();
    Serial.println("Border detected on the left, turning right");
    turnRight();  
    delay(500);  //sim turn time
  }
  
  // If the right IR sensor detects a border 
  else if (rightIR == LOW) {
    stopMoving();
    Serial.println("Border detected on the right, turning left");
    turnLeft();  
    delay(500);  
  }

  delay(1000); 

  static int counter = 0;
  counter++;
  
  if (counter % 4 == 0) {
    dummyLeftIR = LOW;  // Simulate the left IR sensor detecting a border
    dummyRightIR = HIGH;
  } else if (counter % 4 == 2) {
    dummyLeftIR = HIGH;
    dummyRightIR = LOW;  // Simulate the right IR sensor detecting a border
  } else {
    dummyLeftIR = HIGH;
    dummyRightIR = HIGH;  // No borders detected
  }
}

