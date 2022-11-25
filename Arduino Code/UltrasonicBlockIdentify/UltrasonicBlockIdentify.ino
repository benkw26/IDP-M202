#include <HCSR04.h>
#include <Servo.h>
#define LEFT_SERVO 9
#define RIGHT_SERVO 10

Servo left_servo;
Servo right_servo;
UltraSonicDistanceSensor distanceSensorFront(11, 8);


void setup() {
  left_servo.attach(LEFT_SERVO);
  right_servo.attach(RIGHT_SERVO);
  left_servo.write(80);
  right_servo.write(100);
  // left_servo.write(20);
  // right_servo.write(160);
  Serial.begin(115200);
}

void loop() {
  int block_checking = 0;
  for (int i=0; i<3; i++){
    if (check_block()){
      block_checking++;
    }
    delay(500);
  }
  if (block_checking > 1){
    Serial.println("Low-Density");
  } else {
    Serial.println("High-Density");
  }
}

int check_block(){ //returns true for low-density block
  left_servo.write(80);
  right_servo.write(100);
  int average = 0;
  for (int i=0; i<100; i++){
    int us_val = distanceSensorFront.measureDistanceCm();
    if (us_val < 0){
      average += 100;
    } else {
      average += us_val;
    }
    delay(10);
  }
  average = average/100;
  Serial.println(average);
  left_servo.write(20);
  right_servo.write(160);
  return (average<80);
}