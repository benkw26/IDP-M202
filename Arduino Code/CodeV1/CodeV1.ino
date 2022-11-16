#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3
#define LEFT_LINE 6
#define RIGHT_LINE 4
#define LEFT_CHECK 5
#define RIGHT_CHECK 7

//+++DEFINING MOTORSHIELD+++//
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select motor ports M1 and M2
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);
 
//+++GLOBAL VARIABLES+++//
int target_ticks = 0;
int left_ticks = 0;
int right_ticks = 0;
int state = 0;
bool encoder_running = false;
bool line_running = false;

//+++CONSTANT VARIABLES+++//
const float wheel_radius = 34.85; //mm
const float wheel_sep = 240; //mm
const int line_speed = 4090;

void setup() {
  Serial.begin(115200);

  //+++SETTING UP LINE FOLLOW PINS+++//
  pinMode(LEFT_LINE, INPUT);
  pinMode(RIGHT_LINE, INPUT);
  pinMode(LEFT_CHECK, INPUT);
  pinMode(RIGHT_CHECK, INPUT);
 
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
  LeftMotor->setSpeedFine(0);
  RightMotor->setSpeedFine(0);
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);
  line_start();
}
 
void loop() {
  Serial.print("encoder_running");Serial.println(encoder_running);
  Serial.print("line_running");Serial.println(line_running);
  // switch (state) {
  //   case 0:
  //     enc_move(380);
  //     state++;
  //     break;

  //   case 1:
  //     if (!encoder_running){
  //       enc_pivot(900);
  //       state++;
  //     }
  //     break;
    
  //   case 2:
  //     if (!encoder_running){
  //       line_start();
  //       state++;
  //     }
  //     break;

  //   default:
  //     Serial.print("State not written - State ");Serial.println(state);
  // }

  if (line_running){
    line_update();
  } else if (encoder_running){
    enc_update();
  }
}
 




//+++LINE FOLLOW FUNCTIONS+++///

void line_start(){
  Serial.println("Line Starting");
  line_running = true;
  LeftMotor->setSpeedFine(4000);
  RightMotor->setSpeedFine(4000);
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);
  left_ticks = 0;
  right_ticks = 0;
}

void line_update(){
  delay(200);
  Serial.println("Updating Line");
  if(digitalRead(LEFT_LINE)==0 && digitalRead(RIGHT_LINE)==1) {
    //turn left
    LeftMotor->run(BACKWARD);
    RightMotor->run(FORWARD);
  }
  else if(digitalRead(LEFT_LINE)==1 && digitalRead(RIGHT_LINE)==0){
    //turn right
    LeftMotor->run(FORWARD);
    RightMotor->run(BACKWARD);
  }
  else if(digitalRead(LEFT_LINE)==1 && digitalRead(RIGHT_LINE)==1){
    //keep moving
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
  }
}





//+++ENCODER FUNCTIONS+++///

void enc_move(int distance){ //units in mm
  encoder_running = true;
  target_ticks = abs((int)distance/(2*PI*wheel_radius)*96);
  left_ticks = 0;
  right_ticks = 0;
  Serial.print("Target Ticks: ");Serial.println(target_ticks);
  LeftMotor->setSpeedFine(4090);
  RightMotor->setSpeedFine(4090);
  if (distance > 0){
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
  } else {
    LeftMotor->run(BACKWARD);
    RightMotor->run(BACKWARD);
  }
}
 
void enc_pivot(int angle){ //units in 0.1deg
  encoder_running = true;  
  target_ticks = abs((int)(angle * wheel_sep * PI / 3600)/(2*PI*wheel_radius)*96);
  left_ticks = 0;
  right_ticks = 0;
  Serial.print("Target Ticks: ");Serial.println(target_ticks);
  LeftMotor->setSpeedFine(4090);
  RightMotor->setSpeedFine(4090);
  if (angle > 0) {
    LeftMotor->run(FORWARD);
    RightMotor->run(BACKWARD);
  } else {
    LeftMotor->run(BACKWARD);
    RightMotor->run(FORWARD);
  }
}

void enc_update(){
  //  Update motor speed proportionally 
  int left_motor_speed = constrain(1200+(target_ticks-left_ticks)*75, 0, 4090);
  int right_motor_speed = constrain(1200+(target_ticks-right_ticks)*75, 0, 4090);

  // Slow each motor if it's ahead of the other
  if (left_ticks > right_ticks){
    LeftMotor->setSpeedFine((int)left_motor_speed*0.8);
  } else {
    LeftMotor->setSpeedFine(left_motor_speed);
  }
  if (right_ticks > left_ticks){
    RightMotor->setSpeedFine((int)right_motor_speed*0.8);
  } else {
    RightMotor->setSpeedFine(right_motor_speed);  
  }

  // Stop both motors at the end
  if (left_ticks >= target_ticks && right_ticks >= target_ticks){
    encoder_running = false;
    LeftMotor->setSpeedFine(0);
    RightMotor->setSpeedFine(0);
    LeftMotor->run(RELEASE);
    RightMotor->run(RELEASE);
    target_ticks = 0;
    left_ticks = 0;
    right_ticks = 0;
  }
}

void left_interrupt_routine(){
  left_ticks++;
}
void right_interrupt_routine(){
  right_ticks++;
}
 