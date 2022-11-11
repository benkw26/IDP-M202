#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3
 
//+++DEFINING MOTORSHIELD+++//
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select motor ports M1 and M2
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);
 

int target_ticks = 0;
int left_ticks = 0;
int right_ticks = 0;
 
int left_motor_speed = 0;
int right_motor_speed = 0;

// Dimensions in mm
const float wheel_radius = 34.925;
const float base_radius = 237;
const float wheel_sep = 240;

void setup() {
  Serial.begin(115200);
 
  //+++SETTING UP ENCODER PIN INTERRUPTS+++//
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  attachInterrupt(LEFT_ENCODER, left_interrupt_routine, CHANGE);
  attachInterrupt(RIGHT_ENCODER, right_interrupt_routine, CHANGE);
 
  //+++SETTING UP MOTORSHIELD+++//
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
 
  LeftMotor->setSpeed(0);
  RightMotor->setSpeed(0);
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);
 
  move(100);
 
}
 
void loop() {
  delay(100);
  //  Update motor speed proportionally 
  left_motor_speed = 800+(target_ticks-left_ticks)*75;
  right_motor_speed = 800+(target_ticks-right_ticks)*75;
  Serial.println("Left Ticks: ");Serial.println(left_ticks);
  Serial.println("Right Ticks: ");Serial.println(right_ticks);
  Serial.println("Left Speed: ");Serial.println(left_motor_speed);
  Serial.println("Right Speed: ");Serial.println(right_motor_speed);

  // Stop each motor if it's ahead of the other
  if (left_ticks > right_ticks){
    LeftMotor->setSpeed(0);
  } else {
    LeftMotor->setSpeedFine(left_motor_speed);
  }
  if (right_ticks > left_ticks){
    RightMotor->setSpeed(0);
  } else {
    RightMotor->setSpeedFine(right_motor_speed);  
  }

  // Stop both motors at the end
  if (left_ticks >= target_ticks && right_ticks >= target_ticks){
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    LeftMotor->run(RELEASE);
    RightMotor->run(RELEASE);
    left_motor_speed = 0;
    right_motor_speed = 0;
    target_ticks = 0;
    left_ticks = 0;
    right_ticks = 0;
  }
}
 

void move(int distance){ //units in mm
  target_ticks = abs((int)distance/(2*PI*wheel_radius)*96);
  left_ticks = 0;
  right_ticks = 0;
  left_motor_speed = 4090;
  right_motor_speed = 4090;
  Serial.print("Target Ticks: ");Serial.println(target_ticks);
  LeftMotor->setSpeedFine(left_motor_speed);
  RightMotor->setSpeedFine(right_motor_speed);
  if (distance > 0){
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
  } else {
    LeftMotor->run(BACKWARD);
    RightMotor->run(BACKWARD);
  }
}
 
void pivot(int angle){ //units in 0.1deg
  target_ticks = abs((int)(angle * wheel_sep * PI / 3600)/(2*PI*wheel_radius)*96);
  left_ticks = 0;
  right_ticks = 0;
  left_motor_speed = 4090;
  right_motor_speed = 4090;
  Serial.print("Target Ticks: ");Serial.println(target_ticks);
  LeftMotor->setSpeedFine(left_motor_speed);
  RightMotor->setSpeedFine(right_motor_speed);
  if (angle > 0) {
    LeftMotor->run(FORWARD);
    RightMotor->run(BACKWARD);
  } else {
    LeftMotor->run(BACKWARD);
    RightMotor->run(FORWARD);
  }
}
 
void left_interrupt_routine(){
  left_ticks++;
}
void right_interrupt_routine(){
  right_ticks++;
}
 