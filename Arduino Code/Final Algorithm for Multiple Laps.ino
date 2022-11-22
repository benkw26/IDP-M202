#define LEFT_ENCODER 2
#define RIGHT_ENCODER 3
#define LEFT_LINE 4
#define RIGHT_LINE 5
#define LEFT_CHECK 6
#define RIGHT_CHECK 7
#define LEFT_SERVO 9
#define RIGHT_SERVO 10

//+++DEFINING MOTORSHIELD+++//
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select motor ports M1 and M2
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);

//+++DEFINING UPWARDS DISTANCE SENSOR+++//
#include <VitconSAA1064T.h>
#include <VitconGP2Y0A21YK.h>
using namespace vitcon;
SAA1064T fnd;
GP2Y0A21YK sensor(A0);

//+++DEFINING LEFT FACING ULTRASONIC SENSOR+++//
#include <HCSR04.h>
const byte triggerPin = 13;
const byte echoPin = 12;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

//+++DEFINING SERVOMOTORS+++//
#include <Servo.h>
Servo left_servo;
Servo right_servo;

//+++GLOBAL VARIABLES+++//
int target_ticks = 0;
int left_ticks = 0;
int right_ticks = 0;
int state = 0;
int lap = 0;
int pos = 0;
bool encoder_running = false;
bool line_running = false;
bool in_tunnel = false;
bool high_density = false;

//+++CONSTANT VARIABLES+++//
const float wheel_radius = 34.85; //mm
const float wheel_sep = 240; //mm
const int line_speed = 4090;

void setup() {
  Serial.begin(115200);

  //+++INITIALISING UPWARDS DISTANCE SENSOR+++//
  SAA1064T::Init();

  //+++SETTING UP LINE FOLLOW PINS+++//
  pinMode(LEFT_LINE, INPUT_PULLUP);
  pinMode(RIGHT_LINE, INPUT_PULLUP);
  pinMode(LEFT_CHECK, INPUT_PULLUP);
  pinMode(RIGHT_CHECK, INPUT_PULLUP);
 
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

  //+++SETTING UP SERVOMOTOR+++//
  left_servo.attach(LEFT_SERVO);
  right_servo.attach(RIGHT_SERVO);
  left_servo.write(0);
  right_servo.write(180);
  delay(500);
}
 
void loop() {
  Serial.print("encoder_running");Serial.println(encoder_running);
  Serial.print("line_running");Serial.println(line_running);
  
   switch (state) {
     case 0:
     //Moving straight out of the starting box//
       enc_move(380);
       state++;
       break;

     case 1:
     //Turning right onto the main line//
       if (!encoder_running){
         enc_pivot(900);
         state++;
       }
       break;
    
     case 2:
     //Following the line//
       if (!encoder_running){
         line_start();
         state++;
       }
       break;
    
     case 3:
     //Surpassing the first junction//
      if (digitalRead(RIGHT_CHECK)==0 && digitalRead(LEFT_CHECK)==1){
        line_running = false;
        enc_move(50);
        lap += 1;
        state++;
      }
      break;


    
  while (lap < 4) {
    switch (state) {
      case 1:
    //Following the line//
        if (!encoder_running){
          line_start();
          state++;
        }
        continue;

      case 2:
      /*Surpassing the second junction, moving towards
        the block, opening the servos and picking up the block*/
        if (digitalRead(RIGHT_CHECK)==1 && digitalRead(LEFT_CHECK)==0 && lap == 1){
          line_running = false;
          //MOVE SERVO TO OUTWARDS POSITION
          enc_move(300);
          pickup();
          //STORE BLOCK DENSITY AS HIGHDENSITY = TRUE OR FALSE
          state++;
        }
        else if (digitalRead(RIGHT_CHECK)==1 && digitalRead(LEFT_CHECK)==0 && lap == 2) {
          line_running = false;
          enc_pivot(-900);
          enc_move(20);
          pickup();
          //STORE BLOCK AS HIGH DENSITY = TRUE OR FALSE
          enc_pivot(-1800);
          enc_move(20);
          enc_pivot(-900);
          state++;
        }
        else if (digitalRead(RIGHT_CHECK)==1 && digitalRead(LEFT_CHECK)==0 && lap == 3) {
          line_running = false;
          enc_move(50);
          state++;
        }
        continue;

      case 3:
      //Following the line//
        if (!encoder_running) {
          line_start();
          state++;
        }
        continue;

      case 4:
      //Surpassing the crossroads//
        if (digitalRead(RIGHT_CHECK) == 0 && digitalRead(LEFT_CHECK) == 0) {
          line_running = false;
          enc_move(50);
          state++;
        }
        continue;

      case 5:
      //Following the line//
        if (!encoder_running) {
          line_start();
          state++;
        }
        continue;

      case 6:
      //Surpassing the final far end junction//
        if (digitalRead(RIGHT_CHECK) == 1 && digitalRead(LEFT_CHECK) == 0 && (lap == 1 || lap == 2)) {
          line_running = false;
          enc_move(50);
          state++;
        }
        else if (digitalRead(RIGHT_CHECK) == 1 && digitalRead(LEFT_CHECK) == 0 && lap == 3) {
          line_running = false;
          enc_pivot(-900);
          enc_move(20);
          pickup();
          //STORE BLOCK AS HIGH DENSITY = TRUE OR FALSE
          enc_pivot(-1800);
          enc_move(20);
          enc_pivot(-900);
          state++;
        }
        continue;

      case 7:
      //Following the line//
        if (!encoder_running) {
          line_start();
          state++;
        }
        continue;

      case 8:
      //Detecting and entering the tunnel//
        if (sensor.GetDistance() < 40) {
          line_running = false;
          in_tunnel = true;
          tunnel_movement();
          state++;
        }
        continue;

      case 9:
      //Following the line after the tunnel//
        if (in_tunnel == false) {
          line_start();
          state++;
        }
        continue;
      
      case 10:
      //Going to green if necessary
        if (digitalRead(RIGHT_CHECK) == 0 && digitalRead(LEFT_CHECK) == 1 && high_density == false) {
          line_running = false;
          enc_pivot(900);
          enc_move(100);
          drop();
          enc_pivot(1800);
          enc_move(100);
          enc_pivot(90);
          //MOVE SERVOS TO BACKWARDS POSITION//
          state++;
        }
        else if (digitalRead(RIGHT_CHECK == 0) && digitalRead(LEFT_CHECK) == 1 && high_density == true) {
          line_running = false;
          enc_move(50);
          //MOVE SERVOS TO BACKWARDS POSITION//
          state++;
        }
        continue;
      
      case 11:
      //Following the line//
        if (!encoder_running) {
          line_start();
          state++;
        }
        continue;

      case 12:
      //Surpassing starting junction//
        if (digitalRead(RIGHT_CHECK) == 0 && digitalRead(LEFT_CHECK) == 0) {
          line_running = false;
          enc_move(50);
          state++;
        }
        continue;
      
      case 13:
      //Following the line//
        if (!encoder_running) {
          line_start();
          state++;
        }
        continue;

      case 14:
      //Going to red if necessary
        if (digitalRead(RIGHT_CHECK) == 0 && digitalRead(LEFT_CHECK) == 1 && high_density == true) {
          line_running = false;
          enc_pivot(900);
          enc_move(100);
          drop();
          enc_pivot(1800);
          enc_move(100);
          enc_pivot(900);
          enc_move(100);
          lap += 1;
          //MOVE SERVOS TO BACKWARDS POSITION//
          state++;
        }
        else if (digitalRead(RIGHT_CHECK == 0) && digitalRead(LEFT_CHECK) == 1 && high_density == false) {
          line_running = false;
          enc_move(50);
          lap += 1;
          //MOVE SERVOS TO BACKWARDS POSITION//
          state++;
        }
        continue;
    }
  }

    
      default:
        Serial.print("State not written - State ");Serial.println(state);
   }

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

//+++FUNCTION FOR WHEN IN THE TUNNEL+++//
void tunnel_movement() {
  while (in_tunnel) {
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
    if (distanceSensor.measureDistanceCm() * 10.026 - 10.515 < 75) {
      LeftMotor->run(FORWARD);
      RightMotor->run(BACKWARD);
    }
    else if (distanceSensor.measureDistanceCm() * 10.026 - 10.515 > 95) {
      LeftMotor->run(BACKWARD);
      RightMotor->run(FORWARD);
    }
    else if (sensor.GetDistance() > 40) {
      in_tunnel = false;
    }
    else {
      LeftMotor->run(FORWARD);
      RightMotor->run(FORWARD);
    }
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

//+++BLOCK FUNCTIONS+++//
void pickup() {
  for (pos = 90; pos >= 0; pos -= 1) {
    left_servo.write(pos);
    right_servo.write(180 - pos);
    delay(5);
  }
}

void drop() {
  for (pos = 0; pos <= 90; pos += 1) {
    left_servo.write(pos);
    right_servo.write(180 - pos);
    delay(5);
  }
}