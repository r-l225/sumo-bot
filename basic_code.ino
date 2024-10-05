#include <Ultrasonic.h> 

Ultrasonic ultrasonic(4,3);

#define IR_sensor_front A0  
#define IR_sensor_back A1 
int distance;

void setup() 
{
  Serial.begin(9600);
  delay(5000);  // 5 sec delay
}

void loop()
{
  //read sensors
  int IR_front = analogRead(IR_sensor_front);
  int IR_back  = analogRead(IR_sensor_back);
  distance = ultrasonic.read(); 
  
  ROTATE();  // Start rotation to search
  
  // Stop and move forward if the object is closer than 20cm
  if (distance < 20) {
    //stops rotating
    Stop();
    
    //if in distance
    while (distance < 20)  {
      //approach
      FORWARD();
      distance = ultrasonic.read(); 
      
      // Check IR sensors during movement
      IR_front = analogRead(IR_sensor_front);
      IR_back = analogRead(IR_sensor_back);
      
      // Break the loop if a white line detected
      if (IR_front > 650 || IR_back > 650) { 
        break;
      }
      delay(10);
    }
  }
  
  if (IR_front < 650) {  
    Stop();
    delay(50);
    BACKWARD();
    delay(500); 
  }
  
  if (IR_back < 650) {
    Stop();
    delay(50);
    FORWARD();
    delay(500); 
  }


void FORWARD(int Speed) {
}

void BACKWARD(int Speed) {
}

void ROTATE(int Speed) {
}

void Stop() {
}
