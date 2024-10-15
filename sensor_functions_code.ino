// Arduino/ESP32 IR and US Sensor Code
/*
int IRSensor1 = 9; // connect ir sensor module to Arduino pin 9
int IRSensor2 = 10; // connect ir sensor module to Arduino pin 10
int IRSensor3 = 11; // connect ir sensor module to Arduino pin 11
int IRSensor4 = 12; // connect ir sensor module to Arduino pin 12

int trigPin = 4; // Uno PIN 4
int echoPin = 6; // Uno PIN 6
*/
int IRSensor1 = 34; // connect ir sensor module to ESP32 Feather V2 pin A2
int IRSensor2 = 39; // connect ir sensor module to ESP32 Feather V2 pin A3
int IRSensor3 = 36; // connect ir sensor module to ESP32 Feather V2 pin A4
int IRSensor4 = 4;  // connect ir sensor module to ESP32 Feather V2 pin A5

int trigPin = 4; // connect US sensor TRIGPIN to ESP32 Feather V2 pin A0
int echoPin = 6; // connect US sensor ECHOPIN to ESP32 Feather V2 pin A1


void IR_Sensor_setup(byte IRSensorPIN1, byte IRSensorPIN2, byte IRSensorPIN3, byte IRSensorPIN4){
  pinMode(IRSensorPIN1, INPUT); // IR Sensor pin INPUT
  pinMode(IRSensorPIN2, INPUT);
  pinMode(IRSensorPIN3, INPUT);
  pinMode(IRSensorPIN4, INPUT);
}
void ultra_Sensor_setup(byte TRIG_PIN, byte ECHO_PIN){
  pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the ECHO_PIN as an Input
}
uint8_t IR_Sensor_read(byte IRSensorPIN1, byte IRSensorPIN2, byte IRSensorPIN3, byte IRSensorPIN4){
  int status1 = digitalRead(IRSensorPIN1); // Set the GPIO as Input
  int status2 = digitalRead(IRSensorPIN2);
  int status3 = digitalRead(IRSensorPIN3);
  int status4 = digitalRead(IRSensorPIN4);
  uint8_t status = (status4 <<3) | (status3 <<2) | (status2 <<1) | status1;
  return status;
}
float ultra_Sensor_read(byte TRIG_PIN, byte ECHO_PIN){
  float SOUND_SPEED = 0.034;
  long duration;
  float distanceCm;
  // Clears the TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the TRIG_PIN on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  
  return distanceCm;
}
void setup()
{
  Serial.begin(115200); // Init Serial at 115200 Baud
  IR_Sensor_setup(IRSensor1, IRSensor2, IRSensor3, IRSensor4);
  ultra_Sensor_setup(trigPin, echoPin);
}

void loop()
{
  uint8_t IRsensorStatus = IR_Sensor_read(IRSensor1, IRSensor2, IRSensor3, IRSensor4);
  Serial.println(IRsensorStatus, BIN);
  float ultrasensorDistance = ultra_Sensor_read(trigPin, echoPin);
  Serial.println(ultrasensorDistance);
  delay(100);
}

