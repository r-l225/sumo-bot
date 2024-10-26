#include <DFRobot_BMI160.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Motor control pins
int leftMotorPin1 = 5, leftMotorPin2 = 6;
int rightMotorPin1 = 9, rightMotorPin2 = 10;

// IR Sensor Pins (ESP32 Feather V2)
int IRSensor1 = 34; 
int IRSensor2 = 39; 
int IRSensor3 = 36; 
int IRSensor4 = 4; 

// Shared variables between tasks
float targetAngle = 0;  // Target yaw angle for straight movement
float currentAngle = 0; // Current yaw angle from gyroscope
volatile bool finishLineReached = false; // Flag for endline detection

// BMI160 Gyroscope and I2C address
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

// Mutex for shared data
SemaphoreHandle_t mutex;

// Task handles for dual-core operations
TaskHandle_t Task1;
TaskHandle_t Task2;

void readSensors(void *parameter);
void controlMotors(void *parameter);
void stopMotors();
float getGyroYaw();
uint8_t IR_Sensor_read();
void handleBoundary(uint8_t sensorState);

void setup() {
  Serial.begin(115200);

  // Initialize motor pins
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // Initialize IR sensor pins
  pinMode(IRSensor1, INPUT);
  pinMode(IRSensor2, INPUT);
  pinMode(IRSensor3, INPUT);
  pinMode(IRSensor4, INPUT);

  // Initialize the BMI160 Gyroscope
  if (bmi160.softReset() != BMI160_OK || bmi160.I2cInit(i2c_addr) != BMI160_OK) {
    Serial.println("BMI160 initialization failed!");
    while (1);
  }

  // Set the initial yaw angle from the gyroscope
  targetAngle = getGyroYaw();

  // Create a mutex for shared data
  mutex = xSemaphoreCreateMutex();

  // Create tasks for dual-core operation
  xTaskCreatePinnedToCore(readSensors, "ReadSensors", 10000, NULL, 1, &Task1, 0); // Core 0
  xTaskCreatePinnedToCore(controlMotors, "ControlMotors", 10000, NULL, 1, &Task2, 1); // Core 1
}

void loop() {
  delay(100);
}

// Task on Core 0: Reads the IR sensors and updates flags
void readSensors(void *parameter) {
  while (true) {
    uint8_t sensorState = IR_Sensor_read();

    // Check if the endline is detected
    if (sensorState == 0xC) {
      xSemaphoreTake(mutex, portMAX_DELAY);
      finishLineReached = true;
      xSemaphoreGive(mutex);
    }

    // Update the gyro angle
    xSemaphoreTake(mutex, portMAX_DELAY);
    currentAngle = getGyroYaw();
    xSemaphoreGive(mutex);

    delay(50);
  }
}

// Task on Core 1: Adjusts motors and responds to boundaries
void controlMotors(void *parameter) {
  while (true) {
    // Check if the endline is reached
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (finishLineReached) {
      xSemaphoreGive(mutex);
      stopMotors();
      vTaskDelete(NULL);  // Stop this task
    }
    xSemaphoreGive(mutex);

    // Get the IR sensor state and handle boundaries
    uint8_t sensorState = IR_Sensor_read();
    handleBoundary(sensorState);

    // Adjust motor speeds to maintain a straight path
    float error = currentAngle - targetAngle;
    adjustMotors(error);

    delay(50);
  }
}

// Function to read the IR sensors and return a binary/hex value
uint8_t IR_Sensor_read() {
  int s1 = digitalRead(IRSensor1); // Front-Left
  int s2 = digitalRead(IRSensor2); // Front-Right
  int s3 = digitalRead(IRSensor3); // Back-Left
  int s4 = digitalRead(IRSensor4); // Back-Right

  // Combine sensor states into a 4-bit binary value
  uint8_t state = (s4 << 3) | (s3 << 2) | (s2 << 1) | s1;
  return state;
}

// Function to handle motor actions based on IR sensor state
void handleBoundary(uint8_t sensorState) {
  switch (sensorState) {
    case 0x8:  // Front-Left boundary detected
      reverseAndTurnRight();
      break;
    case 0x4:  // Front-Right boundary detected
      reverseAndTurnLeft();
      break;
    case 0x2:  // Back-Left boundary detected
      turnRight();
      break;
    case 0x1:  // Back-Right boundary detected
      turnLeft();
      break;
    case 0xA:  // Left boundary detected (Front-Left + Back-Left)
      turnRight();
      break;
    case 0x5:  // Right boundary detected (Front-Right + Back-Right)
      turnLeft();
      break;
    case 0xC:  // Endline detected
      stopMotors();
      break;
    default:   // No boundary detected
      moveForward();
      break;
  }
}

// Function to get the yaw angle from the BMI160 gyroscope
float getGyroYaw() {
  int16_t accelGyro[6] = {0};
  if (bmi160.getAccelGyroData(accelGyro) == 0) {
    return accelGyro[2] * 3.14 / 180.0;  // Convert Z-axis data to degrees
  }
  return 0;
}

// Motor control functions
void moveForward() {
  analogWrite(leftMotorPin1, 150);
  analogWrite(rightMotorPin1, 150);
}

void reverseAndTurnRight() {
  analogWrite(leftMotorPin2, 150);
  analogWrite(rightMotorPin1, 100);
}

void reverseAndTurnLeft() {
  analogWrite(rightMotorPin2, 150);
  analogWrite(leftMotorPin1, 100);
}

void turnRight() {
  analogWrite(leftMotorPin1, 150);
  analogWrite(rightMotorPin2, 150);
}

void turnLeft() {
  analogWrite(rightMotorPin1, 150);
  analogWrite(leftMotorPin2, 150);
}

void stopMotors() {
  analogWrite(leftMotorPin1, 0);
  analogWrite(leftMotorPin2, 0);
  analogWrite(rightMotorPin1, 0);
  analogWrite(rightMotorPin2, 0);
}

// Adjust motor speeds based on gyro error
void adjustMotors(float error) {
  int baseSpeed = 150;
  if (error > 0.25) {
    analogWrite(leftMotorPin1, baseSpeed - error * 10);
  } else if (error < -0.25) {
    analogWrite(rightMotorPin1, baseSpeed + error * 10);
  } else {
    moveForward();
  }
}
