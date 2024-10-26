#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <DFRobot_BMI160.h> 

// Gyroscope Variables
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;
float angle = 0;  // Store accumulated yaw angle

// IR Sensor Pins (ESP32 Feather V2)
int IRSensor1 = 34;
int IRSensor2 = 39;
int IRSensor3 = 36;
int IRSensor4 = 4;

// Ultrasonic Sensor Pins
int trigPin = 4;
int echoPin = 6;

// Motor Control Pins
const int L_PWM = 15;
const int L_IN1 = 32;
const int L_IN2 = 14;
const int R_PWM = 33;
const int R_IN1 = 13;
const int R_IN2 = 12;

// Thresholds
const int detectionThreshold = 80;

// State Machine
enum State { SEARCHING, MOVING_FORWARD, AVOID, STOPPED };
volatile State currentState = SEARCHING;

volatile uint8_t boundaryCode = 0xF;
volatile bool opponentDetected = false;

// Task handles
TaskHandle_t Task1;  // Sensor reading task
TaskHandle_t Task2;  // Motor control task

void IR_Sensor_setup();
uint8_t IR_Sensor_read();
void ultra_Sensor_setup();
float ultra_Sensor_read();
void TaskSensors(void *pvParameters);
void TaskNavigation(void *pvParameters);
void forwardMotors();
void rotateMotors();
void stopMotors();
void reverseMotors();
void turnLeft();
void turnRight();
void changeState();
void readGyroData();

void setup() {
  Serial.begin(115200);

  IR_Sensor_setup();
  ultra_Sensor_setup();

  // Initialize the BMI160 gyroscope
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("Gyroscope reset failed.");
    while (1);
  }
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
    Serial.println("Gyroscope initialization failed.");
    while (1);
  }

  // Motor pin setup
  pinMode(L_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  // Create the sensor and navigation tasks
  xTaskCreatePinnedToCore(TaskSensors, "TaskSensors", 10000, NULL, 2, &Task1, 1);  // Core 1 for sensors and state management
  xTaskCreatePinnedToCore(TaskNavigation, "TaskNavigation", 10000, NULL, 1, &Task2, 0);  // Core 0 for motor control
}

void loop() {
}

// Core 1: Sensor data collection and state management
void TaskSensors(void *pvParameters) {
  for (;;) {
    boundaryCode = IR_Sensor_read();
    float distance = ultra_Sensor_read();
    opponentDetected = (distance < detectionThreshold);

    changeState();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Handle state changes based on sensor input
void changeState() {
  if (boundaryCode != 0xF) {
    currentState = AVOID;
  } else if (opponentDetected) {
    currentState = MOVING_FORWARD;
  } else {
    currentState = SEARCHING;
  }
}

// Core 0: Navigation and motor control
void TaskNavigation(void *pvParameters) {
  for (;;) {
    switch (currentState) {
      case SEARCHING:
        rotateMotors();
        break;
      case MOVING_FORWARD:
        forwardMotors(); 
        break;
      case AVOID:
        handleBoundaryMovement();
        break;
      case STOPPED:
        stopMotors();
        break;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Read gyroscope data and update the yaw angle
void readGyroData() {
  int16_t accelGyro[6] = {0};
  if (bmi160.getAccelGyroData(accelGyro) == 0) {
    float yaw = (accelGyro[2] * 3.14 / 180.0 + 0.18) * 9 / 7;
    if (yaw > 0.25) {
      angle += yaw / 3.14;
    } else if (yaw <= 0.25 && yaw >= -0.25) {
      angle += 0;
    } else {
      angle -= abs(yaw / 3.14);
    }
  }
}

// Forward movement with gyroscope correction
void forwardMotors() {
  readGyroData();  // Get the current yaw angle

  if (angle > 1.0) {  // If the robot is veering right
    analogWrite(L_PWM, 255);  // Left motor at full speed
    analogWrite(R_PWM, 200);  // Right motor slightly slower
  } else if (angle < -1.0) {  // If the robot is veering left
    analogWrite(L_PWM, 200);  // Left motor slightly slower
    analogWrite(R_PWM, 255);  // Right motor at full speed
  } else {
    analogWrite(L_PWM, 255);  // Both motors at full speed
    analogWrite(R_PWM, 255);
  }
}

void handleBoundaryMovement() {
  switch (boundaryCode) {
    case 0x7:
      turnRight();
      break;
    case 0xB:
      turnLeft();
      break;
    case 0x3:
      reverseMotors();
      break;
    case 0xC:
      forwardMotors();
      break;
    case 0x5:
      turnRight();
      break;
    case 0xA:
      turnLeft();
      break;
    case 0xF:
      forwardMotors();
      break;
    default:
      stopMotors();
      break;
  }
}

void IR_Sensor_setup() {
  pinMode(IRSensor1, INPUT);
  pinMode(IRSensor2, INPUT);
  pinMode(IRSensor3, INPUT);
  pinMode(IRSensor4, INPUT);
}

uint8_t IR_Sensor_read() {
  int status1 = digitalRead(IRSensor1);
  int status2 = digitalRead(IRSensor2);
  int status3 = digitalRead(IRSensor3);
  int status4 = digitalRead(IRSensor4);
  return (status4 << 3) | (status3 << 2) | (status2 << 1) | status1;
}

void ultra_Sensor_setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

float ultra_Sensor_read() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return (duration * 0.034) / 2;
}

void rotateMotors() {
  analogWrite(L_PWM, 150);
  analogWrite(R_PWM, 150);
}

void stopMotors() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

void reverseMotors() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  analogWrite(L_PWM, 255);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
  analogWrite(R_PWM, 255);
}

void turnLeft() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 150);
}

void turnRight() {
  analogWrite(L_PWM, 150);
  analogWrite(R_PWM, 0);
}
